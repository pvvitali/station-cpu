use defmt::info;
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::TIM3;
use embassy_stm32::timer::qei::Qei;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn encoder_task(qei: Qei<'static, TIM3>, pin_btn: Input<'static>) {
    let mut last_count = qei.count();
    let mut absolute_raw: i32 = 0;

    // Флаг для отслеживания предыдущего состояния кнопки
    let mut btn_was_pressed = false;

    info!("Hardware Encoder (TIM3) task started!");

    loop {
        // Засыпаем на 20 мс. Это наш идеальный программный антидребезг.
        Timer::after(Duration::from_millis(20)).await;

        // --- 1. ПРОВЕРКА ВРАЩЕНИЯ ---
        let current_count = qei.count();
        if current_count != last_count {
            // Безопасно вычисляем разницу с учетом переполнения таймера
            let delta = current_count.wrapping_sub(last_count) as i16;
            absolute_raw += delta as i32;
            last_count = current_count;

            // Выводим позицию (4 тика таймера = 1 щелчок)
            info!("Энкодер: {}", absolute_raw / 4);
        }

        // --- 2. ПРОВЕРКА КНОПКИ ---
        // Так как кнопка замыкает на GND, нажатие - это низкий уровень (is_low)
        let btn_is_pressed = pin_btn.is_low();

        // Срабатываем только в момент перехода из отпущенного в нажатое состояние
        if btn_is_pressed && !btn_was_pressed {
            info!("Кнопка нажата! Значение энкодера: {}", absolute_raw / 4);
        }

        // Запоминаем состояние для следующего такта
        btn_was_pressed = btn_is_pressed;
    }
}
