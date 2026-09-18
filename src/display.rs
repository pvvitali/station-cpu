// 1. Стандартные периферийные типы Embassy
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
//use embassy_time::Timer;

// 2. Типы для дисплея
use display_interface_spi::SPIInterface;
use ssd1309::Builder;
use ssd1309::mode::GraphicsMode;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_5X8}, // <-- Изменили шрифт
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

// --- СТРУКТУРЫ ТЕЛЕМЕТРИИ (Восстановлено из main.rs) ---

#[derive(Clone, Default)]
pub struct Telemetry {
    // Модуль 1
    pub mod1_v: f32,
    pub mod1_i: f32,
    pub mod1_pot: f32,

    // Модуль 2
    pub mod2_v: f32,
    pub mod2_i: f32,
    pub mod2_pot: f32,

    // Статусы
    pub is_door_open: bool,
    pub gsm_signal_percent: u8,
}

pub enum DisplayCmd {
    Update(Telemetry),
    Log(String<32>), // Размер строго 32, как вы выделили в main.rs
}

// Глобальный канал связи на 4 сообщения (можно увеличить при необходимости)
pub static DISPLAY_CHANNEL: Channel<ThreadModeRawMutex, DisplayCmd, 4> = Channel::new();

#[embassy_executor::task]
pub async fn display_task(
    spi: Spi<'static, Blocking, Master>,
    dc: Output<'static>,
    cs: Output<'static>, // Возвращаем cs сюда!
    mut rst: Output<'static>,
) {
    // Версия 0.4.1 принимает ровно 3 аргумента и сама управляет CS!
    let interface = SPIInterface::new(spi, dc, cs);

    // Никаких with_size, просто Builder
    let mut display: GraphicsMode<_> = Builder::new().connect(interface).into();

    //Timer::after_secs(1).await;

    // Инициализация
    display.reset(&mut rst, &mut embassy_time::Delay).unwrap();
    display.init().unwrap();
    display.clear();
    display.flush().unwrap();

    // Используем новый шрифт 5x8
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X8)
        .text_color(BinaryColor::On)
        .build();

    loop {
        let cmd = DISPLAY_CHANNEL.receive().await;
        display.clear(); // Очищаем экран

        match cmd {
            DisplayCmd::Log(msg) => {
                // Координата Y=8 — это базовая линия для первой строки шрифта 5x8
                Text::new(msg.as_str(), Point::new(0, 8), text_style)
                    .draw(&mut display)
                    .unwrap();
            }
            DisplayCmd::Update(data) => {
                let mut buf: heapless::String<256> = heapless::String::new();

                // Формируем текст
                core::writeln!(&mut buf, "M1: {:.1}V {:.1}A", data.mod1_v, data.mod1_i).unwrap();
                core::writeln!(&mut buf, "Pot1: {:.3}V", data.mod1_pot).unwrap();

                core::writeln!(&mut buf, "M2: {:.1}V {:.1}A", data.mod2_v, data.mod2_i).unwrap();
                core::writeln!(&mut buf, "Pot2: {:.3}V", data.mod2_pot).unwrap();

                // Чуть-чуть сократили пробелы, чтобы выглядело аккуратнее
                let door = if data.is_door_open { "OPEN" } else { "CLS" };
                core::write!(&mut buf, "GSM:{}% Door:{}", data.gsm_signal_percent, door).unwrap();

                // Рисуем с Y=8
                Text::new(buf.as_str(), Point::new(0, 8), text_style)
                    .draw(&mut display)
                    .unwrap();
            }
        }
        display.flush().unwrap();
    }
}
