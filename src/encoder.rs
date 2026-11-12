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

    // --- НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ АВТО-ОГРАНИЧЕНИЯ ТОКА ---
    let mut last_duty_change_at = embassy_time::Instant::now();
    let mut current_locked = true; // При загрузке ток зафиксирован из флеша
    let mut last_auto_mode = crate::AUTO_MODE.load(Ordering::Relaxed);

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
            let hardware_duty = ((logical_duty as u64 * max_duty as u64) / 1001) as u32;

            pwm.ch1().set_duty_cycle(hardware_duty);
            PWM1_ACTUAL.store(logical_duty, Ordering::Relaxed);

            // // Таймер для ручного энкодера
            // if !skip_save {
            //     crate::FLASH_SAVE_COUNTDOWN.store(10, Ordering::Relaxed);
            // }

            // --- СНИМАЕМ ОГРАНИЧЕНИЕ ТОКА ПРИ ЛЮБОМ ДВИЖЕНИИ НАПРЯЖЕНИЯ ---
            last_duty_change_at = embassy_time::Instant::now();
            if current_locked {
                current_locked = false;
                crate::PWM2_ACTUAL.store(950, Ordering::Relaxed); // Отпускаем на 95%
                let pwm2_hw = ((950_u64 * max_duty as u64) / 1000) as u32;
                pwm.ch2().set_duty_cycle(pwm2_hw);
                info!("Ограничение тока снято (95%) для регулировки напряжения");
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

        // 5. --- АВТОМАТИЧЕСКАЯ ФИКСАЦИЯ ТОКА И АВТО-РЕЖИМ ---
        let auto_mode = crate::AUTO_MODE.load(Ordering::Relaxed);

        if auto_mode && !last_auto_mode {
            // Включили авто-режим (удержание потенциала)
            let pwm2_limit = 333; // 10 Ампер защиты
            crate::PWM2_ACTUAL.store(pwm2_limit, Ordering::Relaxed);
            let pwm2_hw = ((pwm2_limit as u64 * max_duty as u64) / 1000) as u32;
            pwm.ch2().set_duty_cycle(pwm2_hw);
            info!("Авто-режим: Ограничение тока жестко установлено на 10А");
            current_locked = true;
        } else if !auto_mode && last_auto_mode {
            // Выключили авто-режим - отпускаем ток для ручной настройки
            last_duty_change_at = embassy_time::Instant::now();
            current_locked = false;
            let pwm2_limit = 950;
            crate::PWM2_ACTUAL.store(pwm2_limit, Ordering::Relaxed);
            let pwm2_hw = ((pwm2_limit as u64 * max_duty as u64) / 1000) as u32;
            pwm.ch2().set_duty_cycle(pwm2_hw);
            info!("Выход из авто-режима: Ограничение тока снято (95%)");
        }
        last_auto_mode = auto_mode;

        // Если 10 секунд не было регулировки напряжения - "замораживаем" ток
        if !current_locked && !auto_mode && last_duty_change_at.elapsed().as_secs() >= 10 {
            current_locked = true;

            // Читаем фактический ток в трубе
            let current_i_mv = crate::I1_ACTUAL_MV.load(Ordering::Relaxed);
            let current_amps = (current_i_mv as f32) / 1000.0;

            // Запас: +10% от текущего + 0.5А защиты от шумов на малых токах
            let limit_amps = (current_amps * 1.1) + 0.5;

            // Перевод в ШИМ (1000 ШИМ = 30А -> 33.33 ШИМ на 1А)
            let limit_pwm = ((limit_amps * 33.33) as u32).clamp(0, 950);

            crate::PWM2_ACTUAL.store(limit_pwm, Ordering::Relaxed);
            let pwm2_hw = ((limit_pwm as u64 * max_duty as u64) / 1000) as u32;
            pwm.ch2().set_duty_cycle(pwm2_hw);

            info!(
                "Ток зафиксирован на {} мА (PWM2: {})",
                (limit_amps * 1000.0) as i32,
                limit_pwm
            );

            // Сохраняем новые настройки во флеш (Это гарантирует сохранение
            // даже команд с сервера, минуя спам промежуточных шагов доводчика!)
            crate::SAVE_FLASH_REQUEST.store(true, Ordering::Relaxed);
        }
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
