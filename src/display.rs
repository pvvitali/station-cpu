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
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10}, // Увеличили шрифт
    pixelcolor::BinaryColor,
    prelude::*,
    // Добавляем инструменты для рисования линий и прямоугольников:
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text, TextStyleBuilder},
};

// --- СТРУКТУРЫ ТЕЛЕМЕТРИИ (Восстановлено из main.rs) ---

#[derive(Clone, Default)]
pub struct Telemetry {
    pub mod1_v: f32,
    pub mod1_i: f32,
    pub mod2_v: f32,
    pub mod2_i: f32,
    pub p: f32, // Оставили только один потенциал
    pub is_door_open: bool,
    pub gsm_signal_percent: u8,
    pub diag_msg: heapless::String<32>, // Новое поле для сообщения внизу экрана
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

    // 1. Стиль самих букв (используем новый шрифт 6x10)
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10) // Изменили с FONT_5X8 на FONT_6X10
        .text_color(BinaryColor::On)
        .build();

    // 2. Стиль макета (Прибиваем координаты строго к ВЕРХНЕМУ краю)
    // Именно эту переменную потерял компилятор
    let text_layout = TextStyleBuilder::new().baseline(Baseline::Top).build();

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
                let mut buf: heapless::String<64> = heapless::String::new();

                // Стиль для рисования линий и пустых рамок
                let stroke_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
                // Стиль для заливки (сплошные прямоугольники)
                let fill_style = PrimitiveStyle::with_fill(BinaryColor::On);

                // --- ЛЕВАЯ КОЛОНКА (ДАТЧИКИ) ---

                // Модуль 1
                buf.clear();
                core::write!(&mut buf, "U= {:.1}V", data.mod1_v).unwrap();
                Text::with_text_style(buf.as_str(), Point::new(0, 0), text_style, text_layout)
                    .draw(&mut display)
                    .unwrap();
                buf.clear();
                core::write!(&mut buf, "I= {:.2}A", data.mod1_i).unwrap();
                Text::with_text_style(buf.as_str(), Point::new(0, 10), text_style, text_layout)
                    .draw(&mut display)
                    .unwrap();

                // Разделитель 1
                Line::new(Point::new(0, 20), Point::new(50, 20))
                    .into_styled(stroke_style)
                    .draw(&mut display)
                    .unwrap();

                // Модуль 2
                buf.clear();
                core::write!(&mut buf, "U= {:.1}V", data.mod2_v).unwrap();
                Text::with_text_style(buf.as_str(), Point::new(0, 22), text_style, text_layout)
                    .draw(&mut display)
                    .unwrap();
                buf.clear();
                core::write!(&mut buf, "I= {:.2}A", data.mod2_i).unwrap();
                Text::with_text_style(buf.as_str(), Point::new(0, 32), text_style, text_layout)
                    .draw(&mut display)
                    .unwrap();

                // Разделитель 2
                Line::new(Point::new(0, 42), Point::new(50, 42))
                    .into_styled(stroke_style)
                    .draw(&mut display)
                    .unwrap();

                // Потенциал
                buf.clear();
                core::write!(&mut buf, "P= {:.2}V", data.p).unwrap();
                Text::with_text_style(buf.as_str(), Point::new(0, 44), text_style, text_layout)
                    .draw(&mut display)
                    .unwrap();

                // --- ПРАВАЯ КОЛОНКА (ИКОНКИ) ---

                // // 1. Иконка Антенны (в виде классических столбиков связи)
                // let bars = match data.gsm_signal_percent {
                //     0..=20 => 1,
                //     21..=50 => 2,
                //     51..=80 => 3,
                //     _ => 4,
                // };
                // for i in 0..4 {
                //     let h = (i + 1) * 2; // Высота столбиков: 2, 4, 6, 8 пикселей
                //     let x_pos = 75 + i * 4;
                //     let y_pos = 10 - h;
                //     if i < bars {
                //         // Закрашенный столбик (есть сигнал)
                //         Rectangle::new(Point::new(x_pos, y_pos), Size::new(2, h as u32))
                //             .into_styled(fill_style)
                //             .draw(&mut display)
                //             .unwrap();
                //     } else {
                //         // Пустой контур столбика (нет сигнала)
                //         Rectangle::new(Point::new(x_pos, y_pos), Size::new(2, h as u32))
                //             .into_styled(stroke_style)
                //             .draw(&mut display)
                //             .unwrap();
                //     }
                // }
                // 1. Иконка Антенны (в виде классических столбиков связи)
                let bars = match data.gsm_signal_percent {
                    0..=20 => 1,
                    21..=50 => 2,
                    51..=80 => 3,
                    _ => 4,
                };
                for i in 0..4 {
                    // Увеличили высоту: 3, 6, 9, 12 пикселей
                    let h = (i + 1) * 3;
                    let x_pos = 112 + i * 4;
                    // Базовая линия теперь на Y=12
                    let y_pos = 12 - h;

                    if i < bars {
                        // Закрашенный столбик (ширина 3 пикселя)
                        Rectangle::new(Point::new(x_pos, y_pos), Size::new(3, h as u32))
                            .into_styled(fill_style)
                            .draw(&mut display)
                            .unwrap();
                    } else {
                        // Пустой контур (при ширине 3 пикселя внутри будет видна 1 пиксель пустоты)
                        Rectangle::new(Point::new(x_pos, y_pos), Size::new(3, h as u32))
                            .into_styled(stroke_style)
                            .draw(&mut display)
                            .unwrap();
                    }
                }
                // Текст уровня сигнала
                buf.clear();
                core::write!(&mut buf, "{}", data.gsm_signal_percent).unwrap();
                Text::with_text_style(buf.as_str(), Point::new(98, 2), text_style, text_layout)
                    .draw(&mut display)
                    .unwrap();

                // 2. Иконка Двери
                if data.is_door_open {
                    // Иконка открытой двери (смещенный контур)
                    Rectangle::new(Point::new(120, 22), Size::new(8, 12))
                        .into_styled(stroke_style)
                        .draw(&mut display)
                        .unwrap();
                    Line::new(Point::new(120, 22), Point::new(114, 19))
                        .into_styled(stroke_style)
                        .draw(&mut display)
                        .unwrap();
                    Line::new(Point::new(120, 34), Point::new(114, 36))
                        .into_styled(stroke_style)
                        .draw(&mut display)
                        .unwrap();
                    Line::new(Point::new(114, 19), Point::new(114, 36))
                        .into_styled(stroke_style)
                        .draw(&mut display)
                        .unwrap();
                    // Дверная ручка (на распахнутой створке)
                    Rectangle::new(Point::new(115, 27), Size::new(2, 2))
                        .into_styled(fill_style)
                        .draw(&mut display)
                        .unwrap();
                    // Text::with_text_style("OPEN", Point::new(90, 24), text_style, text_layout)
                    //     .draw(&mut display)
                    //     .unwrap();
                } else {
                    // Иконка закрытой двери (сплошной контур с ручкой)
                    Rectangle::new(Point::new(116, 22), Size::new(8, 12))
                        .into_styled(stroke_style)
                        .draw(&mut display)
                        .unwrap();
                    // Дверная ручка
                    Rectangle::new(Point::new(121, 27), Size::new(2, 2))
                        .into_styled(fill_style)
                        .draw(&mut display)
                        .unwrap();
                    // Text::with_text_style("LOCKED", Point::new(90, 24), text_style, text_layout)
                    //     .draw(&mut display)
                    //     .unwrap();
                }

                // --- НИЖНЯЯ ПАНЕЛЬ (ДИАГНОСТИКА) ---

                // Отделяем низ линией через весь экран
                // Line::new(Point::new(0, 53), Point::new(65, 53))
                //     .into_styled(stroke_style)
                //     .draw(&mut display)
                //     .unwrap();

                // Выводим диагностическое сообщение
                Text::with_text_style(
                    data.diag_msg.as_str(),
                    Point::new(0, 54),
                    text_style,
                    text_layout,
                )
                .draw(&mut display)
                .unwrap();
            }
        }
        display.flush().unwrap();
    }
}
