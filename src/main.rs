#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;

// Макрос инициализирует таблицу векторов и планировщик задач (Executor)
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Инициализация всей периферии и системы тактирования (по умолчанию 16 МГц)
    let p = embassy_stm32::init(Default::default());

    // Настраиваем пин PB2 как Push-Pull выход
    // Начальное состояние - Low (выключен), скорость порта - Low (хватит для светодиода)
    let mut led = Output::new(p.PB2, Level::Low, Speed::Low);

    defmt::info!("Embassy: Blinky project started!");

    loop {
        // Зажигаем светодиод
        led.set_high();
        defmt::info!("LED ON");

        // Магия асинхронности: усыпляем процессор на 500 мс.
        // Ядро не крутит пустой цикл! Оно ждет прерывания от аппаратного таймера.
        Timer::after_millis(500).await;

        // Гасим светодиод
        led.set_low();
        defmt::info!("LED OFF");

        Timer::after_millis(500).await;
    }
}
