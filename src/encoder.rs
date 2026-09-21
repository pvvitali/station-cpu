use defmt::info;
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::{TIM3, TIM4};
use embassy_stm32::timer::qei::Qei;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn encoder_task(
    qei: Qei<'static, TIM3>,
    pin_btn: Input<'static>,
    mut pwm: SimplePwm<'static, TIM4>,
    max_duty: u32,
) {
    let mut last_count = qei.count();
    let mut absolute_raw: i32 = 0;
    let mut last_ui_position: u32 = 0;

    let limit_min: i32 = 0;
    let limit_max: i32 = (max_duty * 4) as i32;
    let mut btn_was_pressed = false;

    info!("Encoder task with speed acceleration started!");

    loop {
        Timer::after(Duration::from_millis(20)).await;

        let current_count = qei.count();
        if current_count != last_count {
            let delta = current_count.wrapping_sub(last_count) as i16;
            last_count = current_count;

            // 1. ВЫЧИСЛЯЕМ СКОРОСТЬ
            // Делим на 4, чтобы получить количество реальных щелчков за 20 мс.
            // Метод max(1) спасает от умножения на 0 при случайном дребезге в 1-2 тика.
            let speed_clicks = (delta.abs() / 4).max(1) as i32;

            // 2. ФОРМИРУЕМ МНОЖИТЕЛЬ
            // Базовый шаг 10. Если крутануть быстро (2+ щелчка), множитель пропорционально растет.
            let step_multiplier = 10 * speed_clicks;

            // 3. ПРИМЕНЯЕМ УСКОРЕНИЕ
            let accelerated_delta = (delta as i32) * step_multiplier;
            absolute_raw = (absolute_raw + accelerated_delta).clamp(limit_min, limit_max);

            let ui_position = (absolute_raw / 4) as u32;

            if ui_position != last_ui_position {
                pwm.ch1().set_duty_cycle(ui_position);

                info!(
                    "U1 PWM: {} | Скорость: {} щелчков/такт | Шаг x{}",
                    ui_position, speed_clicks, step_multiplier
                );

                last_ui_position = ui_position;
            }
        }

        let btn_is_pressed = pin_btn.is_low();
        if btn_is_pressed && !btn_was_pressed {
            info!("Кнопка нажата!");
        }
        btn_was_pressed = btn_is_pressed;
    }
}
