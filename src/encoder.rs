// Уберите импорт Channel::*, он больше не нужен
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

    let limit_min: i32 = 0;
    let limit_max: i32 = (max_duty * 4) as i32;

    let mut btn_was_pressed = false;

    info!("Encoder task started! Max duty = {}", max_duty);

    loop {
        Timer::after(Duration::from_millis(20)).await;

        let current_count = qei.count();
        if current_count != last_count {
            let delta = current_count.wrapping_sub(last_count) as i16;

            absolute_raw = (absolute_raw + delta as i32).clamp(limit_min, limit_max);
            last_count = current_count;

            let ui_position = (absolute_raw / 4) as u32;

            // ВЕРНОЕ ОБРАЩЕНИЕ В 0.6: Берем канал "на лету" и задаем скважность
            pwm.ch1().set_duty_cycle(ui_position);
            pwm.ch2().set_duty_cycle(ui_position);
            pwm.ch3().set_duty_cycle(ui_position);
            pwm.ch4().set_duty_cycle(ui_position);

            info!(
                "U1 PWM: {} / {} ({}%)",
                ui_position,
                max_duty,
                (ui_position as f32 / max_duty as f32 * 100.0) as u32
            );
        }

        let btn_is_pressed = pin_btn.is_low();
        if btn_is_pressed && !btn_was_pressed {
            info!("Кнопка нажата!");
        }
        btn_was_pressed = btn_is_pressed;
    }
}
