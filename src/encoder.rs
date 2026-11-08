use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::{TIM3, TIM4};
use embassy_stm32::timer::qei::Qei;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};

// Подключаем глобальные переменные из main.rs
use crate::{PWM_CHANNEL, PWM1_ACTUAL};

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

        // 1. Проверяем команды из канала (от DWIN или GSM)
        if let Ok(new_pwm_val) = PWM_CHANNEL.try_receive() {
            absolute_raw = (new_pwm_val as i32).clamp(0, 1001) * 4;
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

            //команда для записи во флеш!
            // Сбрасываем таймер на 10 секунд при каждом изменении (щелчке)
            crate::FLASH_SAVE_COUNTDOWN.store(10, Ordering::Relaxed);

            info!(
                "PWM Обновлен | Логика: {}/1001 | Аппаратный: {}/{}",
                logical_duty, hardware_duty, max_duty
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
