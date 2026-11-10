use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::{TIM3, TIM4};
use embassy_stm32::timer::qei::Qei;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};

// Подключаем глобальные переменные из main.rs
use crate::{PWM_CHANNEL, PWM1_ACTUAL};

use crate::{ADJUST_CHANNEL, AdjustCmd};

#[embassy_executor::task]
pub async fn encoder_task(
    qei: Qei<'static, TIM3>,
    pin_btn: Input<'static>,
    mut pwm: SimplePwm<'static, TIM4>,
    max_duty: u32,
) {
    let mut last_count = qei.count();

    // БЕРЕМ ЗНАЧЕНИЕ, ВОССТАНОВЛЕННОЕ ИЗ ФЛЕШ-ПАМЯТИ
    let initial_logical = PWM1_ACTUAL.load(Ordering::Relaxed) as i32;
    let mut absolute_raw: i32 = initial_logical.clamp(0, 1001) * 4;

    let limit_min: i32 = 0;
    let limit_max: i32 = 1001 * 4;
    let mut btn_was_pressed = false;

    info!("Encoder & PWM Manager task started!");

    loop {
        Timer::after(Duration::from_millis(20)).await;
        let mut duty_changed = false;
        let mut skip_save = false; // Флаг запрета записи во флеш

        // 1. Проверяем команды из канала (от DWIN или GSM)
        if let Ok(new_pwm_val) = PWM_CHANNEL.try_receive() {
            // Если выставлен 31-й бит, значит это временная команда (Вкл/Выкл), писать во флеш нельзя!
            if (new_pwm_val & (1 << 31)) != 0 {
                skip_save = true;
            }
            // Отбрасываем этот скрытый бит, чтобы получить чистый ШИМ
            let val = new_pwm_val & !(1 << 31);

            absolute_raw = (val as i32).clamp(0, 1001) * 4;
            duty_changed = true;
        }

        // 2. Проверяем физический энкодер
        let current_count = qei.count();
        if current_count != last_count {
            let delta = current_count.wrapping_sub(last_count) as i16;
            last_count = current_count;

            let speed_clicks = (delta.abs() / 4).max(1) as i32;
            let step_multiplier = 2 * speed_clicks;
            let accelerated_delta = (delta as i32) * step_multiplier;

            let next_raw = absolute_raw + accelerated_delta;
            let clamped_raw = next_raw.clamp(limit_min, limit_max);

            if clamped_raw != absolute_raw {
                absolute_raw = clamped_raw;
                duty_changed = true;
                skip_save = false; // Если покрутили ручку руками — 100% пишем во флеш!

                // --- ОТКЛЮЧАЕМ АВТО РЕЖИМ ПРИ РУЧНОМ ВРАЩЕНИИ ---
                if crate::AUTO_MODE.load(Ordering::Relaxed) {
                    crate::AUTO_MODE.store(false, Ordering::Relaxed);
                    info!("Ручное вмешательство: Авто-режим отключен!");
                }
            }
        }

        // 3. Применяем новое значение к железу, если оно изменилось
        if duty_changed {
            let logical_duty = (absolute_raw / 4) as u32;

            // Масштабируем 0..1001 в 0..max_duty
            let hardware_duty = ((logical_duty as u64 * max_duty as u64) / 1001) as u32;

            pwm.ch1().set_duty_cycle(hardware_duty);

            // Публикуем новое логическое значение для отправки в GSM и обратно в DWIN
            PWM1_ACTUAL.store(logical_duty, Ordering::Relaxed);

            // Сбрасываем таймер на сохранение ТОЛЬКО если нет флага skip_save
            if !skip_save {
                crate::FLASH_SAVE_COUNTDOWN.store(10, Ordering::Relaxed);
            }

            info!(
                "PWM Обновлен | Лог: {}/1001 | Аппарат: {}/{} (Запись во флеш: {})",
                logical_duty, hardware_duty, max_duty, !skip_save
            );
        }

        // 4. Обработка кнопки
        let btn_is_pressed = pin_btn.is_low();
        if btn_is_pressed && !btn_was_pressed {
            info!("Кнопка энкодера нажата!");
            // Позже сюда можно добавить логику сброса ШИМ или перехода в меню
        }
        btn_was_pressed = btn_is_pressed;
    }
}

#[embassy_executor::task]
pub async fn voltage_adjuster_task() {
    info!("Voltage Adjuster task started!");

    loop {
        // Ждем команду на изменение напряжения из GSM или меню
        let cmd = ADJUST_CHANNEL.receive().await;

        let current_u_mv = crate::U1_ACTUAL_MV.load(Ordering::Relaxed);

        // Определяем цель (в милливольтах) и начальный крупный шаг ШИМа
        let (target_mv, step_coarse) = match cmd {
            AdjustCmd::Up1V => (current_u_mv + 1000, 17),
            AdjustCmd::Down1V => (current_u_mv.saturating_sub(1000).max(0), -17),
            AdjustCmd::Up10V => (current_u_mv + 10_000, 150),
            AdjustCmd::Down10V => (current_u_mv.saturating_sub(10_000).max(0), -150),
            AdjustCmd::Up0_1V => (current_u_mv + 100, 2),
            AdjustCmd::Down0_1V => (current_u_mv.saturating_sub(100).max(0), -2),
        };

        info!(
            "Доводчик: Старт. Текущее: {} мВ, Цель: {} мВ",
            current_u_mv, target_mv
        );

        // 1. Делаем стартовый крупный прыжок ШИМа
        let mut current_pwm = crate::PWM1_ACTUAL.load(Ordering::Relaxed) as i32;
        current_pwm = (current_pwm + step_coarse).clamp(0, 1001);

        // Отправляем новое значение ШИМ в канал
        let _ = crate::PWM_CHANNEL.try_send(current_pwm as u32);

        // Ждем 500 мс (аналог HAL_Delay) для зарядки конденсаторов
        Timer::after(Duration::from_millis(500)).await;

        // 2. Цикл плавной доводки (динамический регулятор)
        let mut timeout_guard = 0; // Защита от вечного цикла

        loop {
            let u_mv = crate::U1_ACTUAL_MV.load(Ordering::Relaxed);
            let diff = target_mv - u_mv;
            let abs_diff = diff.abs();

            // Если разница меньше 60 мВ — цель достигнута!
            if abs_diff <= 60 || timeout_guard > 50 {
                info!(
                    "Доводчик: Работа завершена! Финишное напряжение: {} мВ",
                    u_mv
                );

                // Даем сигнал GSM отправить обновленные данные
                crate::FORCE_TELEMETRY_UPDATE.store(true, Ordering::Relaxed);
                break;
            }

            // УМНЫЙ ШАГ: чем дальше цель, тем быстрее крутим ШИМ!
            // Это делает код независимым от типа блока питания (36V или 48V)
            let step = if abs_diff > 3000 {
                25 // До цели больше 3В -> делаем огромные шаги
            } else if abs_diff > 1000 {
                10 // До цели больше 1В -> средние шаги
            } else if abs_diff > 300 {
                3 // До цели больше 0.3В -> мелкие шаги
            } else {
                1 // Ювелирная доводка у самого финиша
            };

            // Направляем шаг в нужную сторону (+ или -)
            let signed_step = if diff > 0 { step } else { -step };
            current_pwm = (current_pwm + signed_step).clamp(0, 1001);

            let _ = crate::PWM_CHANNEL.try_send((current_pwm as u32) | (1 << 31));

            timeout_guard += 1;

            // Даем напряжению 200 мс на стабилизацию (быстрее, чем 300 мс)
            Timer::after(Duration::from_millis(200)).await;
        }
    }
}

#[embassy_executor::task]
pub async fn auto_regulation_task() {
    info!("Auto Regulation task started!");

    loop {
        Timer::after(Duration::from_secs(3)).await;

        if !crate::AUTO_MODE.load(Ordering::Relaxed) {
            continue;
        }

        // БЕРЕМ МОДУЛИ (АБСОЛЮТНЫЕ ЗНАЧЕНИЯ) ДЛЯ КОРРЕКТНОЙ МАТЕМАТИКИ
        let target_mag = crate::AUTO_TARGET_MV.load(Ordering::Relaxed).abs();
        let actual_mag = crate::P1_ACTUAL_MV.load(Ordering::Relaxed).abs();

        let diff = target_mag - actual_mag;
        let abs_diff = diff.abs();

        // Гистерезис: если напряжение в пределах ±30 мВ от цели, не дергаем ШИМ
        if abs_diff <= 30 {
            continue;
        }

        let mut current_pwm = crate::PWM1_ACTUAL.load(Ordering::Relaxed) as i32;

        let step = if abs_diff > 500 {
            5
        } else if abs_diff > 150 {
            2
        } else {
            1
        };

        // Логика направления:
        // Если фактический потенциал по модулю МЕНЬШЕ целевого (например, 800мВ вместо нужных 1250мВ),
        // значит труба недостаточно защищена, и нам нужно УВЕЛИЧИТЬ мощность станции (ШИМ +).
        if actual_mag < target_mag {
            current_pwm += step;
        } else {
            current_pwm -= step; // Перестарались, нужно убавить
        }

        current_pwm = current_pwm.clamp(0, 1001);

        // Отправляем новое значение ШИМ со скрытым битом, чтобы сберечь флеш-память!
        let _ = crate::PWM_CHANNEL.try_send((current_pwm as u32) | (1 << 31));
    }
}
