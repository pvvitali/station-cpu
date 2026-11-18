use core::sync::atomic::Ordering;
use defmt::info;
use embassy_futures::join::join;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{Peri, bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;

// ИСПРАВЛЕНИЕ: Используем правильные аппаратные имена прерываний для STM32F4 (STREAM вместо CH)
bind_interrupts!(struct Irqs {
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    DMA2_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_STREAM6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH6>;
});

async fn dwin_write_float<W: Write>(uart: &mut W, addr: u16, value: f32) {
    let val_bits = value.to_bits();
    let buf = [
        0x5A,
        0xA5,
        0x07,
        0x82,
        (addr >> 8) as u8,
        (addr & 0xFF) as u8,
        (val_bits >> 24) as u8,
        (val_bits >> 16) as u8,
        (val_bits >> 8) as u8,
        (val_bits & 0xFF) as u8,
    ];
    let _ = uart.write_all(&buf).await;
}

async fn dwin_write_int16<W: Write>(uart: &mut W, addr: u16, value: u16) {
    let buf = [
        0x5A,
        0xA5,
        0x05, // Длина оставшейся части пакета
        0x82, // Команда записи
        (addr >> 8) as u8,
        (addr & 0xFF) as u8,
        (value >> 8) as u8,
        (value & 0xFF) as u8,
    ];
    let _ = uart.write_all(&buf).await;
}

async fn dwin_write_int32<W: embedded_io_async::Write>(uart: &mut W, addr: u16, value: u32) {
    let buf = [
        0x5A,
        0xA5,
        0x07, // Длина оставшейся части пакета
        0x82, // Команда записи
        (addr >> 8) as u8,
        (addr & 0xFF) as u8,
        (value >> 24) as u8,
        (value >> 16) as u8,
        (value >> 8) as u8,
        (value & 0xFF) as u8,
    ];
    let _ = uart.write_all(&buf).await;
}

#[embassy_executor::task]
pub async fn dwin_task(
    usart6: Peri<'static, peripherals::USART6>,
    rx_pin: Peri<'static, peripherals::PC7>,
    tx_pin: Peri<'static, peripherals::PC6>,
    rx_dma: Peri<'static, peripherals::DMA2_CH1>,
    tx_dma: Peri<'static, peripherals::DMA2_CH6>,
) {
    let mut config = Config::default();
    config.baudrate = 115200;

    // ИСПРАВЛЕНИЕ: убрали mut, так как .split() забирает объект по значению
    let dwin_uart = Uart::new(usart6, rx_pin, tx_pin, tx_dma, rx_dma, Irqs, config).unwrap();

    info!("DWIN Display task started (DMA mode: PC7=RX, PC6=TX)");

    let (mut tx, mut rx) = dwin_uart.split();

    let rx_fut = async {
        let mut rx_buf = [0u8; 128];

        loop {
            match rx.read_until_idle(&mut rx_buf).await {
                Ok(len) if len > 0 => {
                    let buf = &rx_buf[..len];
                    let mut i = 0;

                    while i < buf.len() {
                        if i + 1 < buf.len() && buf[i] == 0x5A && buf[i + 1] == 0xA5 {
                            if i + 2 < buf.len() {
                                let pkt_len = buf[i + 2] as usize;
                                let full_pkt_len = 3 + pkt_len;

                                if i + full_pkt_len <= buf.len() {
                                    let pkt = &buf[i..i + full_pkt_len];

                                    if pkt_len >= 6 && pkt[3] == 0x83 {
                                        let addr = ((pkt[4] as u16) << 8) | (pkt[5] as u16);
                                        let data_len = pkt[6] as usize * 2; // Длина данных в байтах

                                        if addr == 0x5032 && data_len == 4 && pkt.len() >= 11 {
                                            // --- ОБРАБОТКА FLOAT (4 байта) ДЛЯ ЦЕЛЕВОГО ПОТЕНЦИАЛА ---
                                            let bits = ((pkt[7] as u32) << 24)
                                                | ((pkt[8] as u32) << 16)
                                                | ((pkt[9] as u32) << 8)
                                                | (pkt[10] as u32);

                                            let target_v = f32::from_bits(bits);
                                            // Берем чистый модуль и переводим Вольты в милливольты
                                            // чтобы 1199.999 превратилось ровно в 1200
                                            // Прибавляем 0.5 для правильного математического округления
                                            let target_mv = (target_v.abs() * 1000.0 + 0.5) as i32;

                                            info!(
                                                "-> DWIN: Целевой потенциал = {} мВ (Введено: {} В)",
                                                target_mv, target_v
                                            );

                                            crate::AUTO_TARGET_MV
                                                .store(target_mv, Ordering::Relaxed);
                                            crate::SAVE_FLASH_REQUEST
                                                .store(true, Ordering::Relaxed);

                                            if crate::AUTO_MODE.load(Ordering::Relaxed) {
                                                crate::FORCE_TELEMETRY_UPDATE
                                                    .store(true, Ordering::Relaxed);
                                            }
                                        } else if data_len == 2 && pkt.len() >= 9 {
                                            // --- СТАНДАРТНАЯ ОБРАБОТКА INT16 (2 байта) ---
                                            let data = ((pkt[7] as u16) << 8) | (pkt[8] as u16);

                                            match addr {
                                                0x5012 => {
                                                    info!("-> Изменение ШИМ: {}", data);
                                                    if crate::AUTO_MODE.load(Ordering::Relaxed) {
                                                        crate::AUTO_MODE
                                                            .store(false, Ordering::Relaxed);
                                                        info!(
                                                            "Ручное вмешательство (DWIN): Авто-режим отключен!"
                                                        );
                                                    }
                                                    let _ =
                                                        crate::PWM_CHANNEL.try_send(data as u32);
                                                }
                                                0x5030 => {
                                                    info!("-> GSM Переключатель (DWIN): {}", data);
                                                    crate::GSM_ON
                                                        .store(data > 0, Ordering::Relaxed);
                                                    crate::SAVE_FLASH_REQUEST
                                                        .store(true, Ordering::Relaxed);
                                                }
                                                0x5031 => {
                                                    let is_auto = data > 0;
                                                    info!("-> DWIN: АВТО-РЕЖИМ = {}", is_auto);
                                                    crate::AUTO_MODE
                                                        .store(is_auto, Ordering::Relaxed);
                                                    crate::SAVE_FLASH_REQUEST
                                                        .store(true, Ordering::Relaxed);
                                                }
                                                0x5038 => {
                                                    let is_limit = data > 0;
                                                    info!("-> DWIN: Лимит 10А = {}", is_limit);
                                                    crate::LIMIT_10A_EN
                                                        .store(is_limit, Ordering::Relaxed);
                                                    crate::SAVE_FLASH_REQUEST
                                                        .store(true, Ordering::Relaxed);
                                                }
                                                0x5039 => {
                                                    let is_mode = data > 0;
                                                    info!("-> DWIN: Регулировка V/I = {}", is_mode);
                                                    crate::V_I_MODE_EN
                                                        .store(is_mode, Ordering::Relaxed);
                                                    crate::SAVE_FLASH_REQUEST
                                                        .store(true, Ordering::Relaxed);
                                                }
                                                _ => {}
                                            }
                                        }
                                    }

                                    // if pkt_len >= 6 && pkt[3] == 0x83 {
                                    //     let addr = ((pkt[4] as u16) << 8) | (pkt[5] as u16);
                                    //     let data = ((pkt[7] as u16) << 8) | (pkt[8] as u16);

                                    //     match addr {
                                    //         0x5012 => {
                                    //             info!("-> Изменение ШИМ: {}", data);

                                    //             // --- ОТКЛЮЧАЕМ АВТО-РЕЖИМ ПРИ РУЧНОМ УПРАВЛЕНИИ С ЭКРАНА ---
                                    //             if crate::AUTO_MODE.load(Ordering::Relaxed) {
                                    //                 crate::AUTO_MODE
                                    //                     .store(false, Ordering::Relaxed);
                                    //                 info!(
                                    //                     "Ручное вмешательство (DWIN): Авто-режим отключен!"
                                    //                 );
                                    //                 // Сохранение во флеш-память автоматически подхватится
                                    //                 // через 10-секундный таймер в задаче энкодера.
                                    //             }

                                    //             // Отправляем значение от экрана в канал управления ШИМ
                                    //             let _ = crate::PWM_CHANNEL.try_send(data as u32);
                                    //         }
                                    //         0x5017 => info!("-> Аккумулятор: {}", data),
                                    //         // Внутри rx_fut, где парсится адрес 0x5030
                                    //         0x5030 => {
                                    //             info!("-> GSM Переключатель (DWIN): {}", data);
                                    //             crate::GSM_ON.store(data > 0, Ordering::Relaxed);
                                    //             // При изменении настроек сразу запрашиваем сохранение
                                    //             crate::SAVE_FLASH_REQUEST
                                    //                 .store(true, Ordering::Relaxed);
                                    //         }
                                    //         //// Если добавишь 0x5014 для ШИМ2 (тока):
                                    //         // 0x5014 => {
                                    //         //     crate::PWM2_ACTUAL
                                    //         //         .store(data as u32, Ordering::Relaxed);
                                    //         //     crate::SAVE_FLASH_REQUEST
                                    //         //         .store(true, Ordering::Relaxed);
                                    //         // }
                                    //         _ => {}
                                    //     }
                                    // }
                                    i += full_pkt_len;
                                    continue;
                                }
                            }
                        }
                        i += 1;
                    }
                }
                _ => {}
            }
        }
    };

    let tx_fut = async {
        loop {
            Timer::after(Duration::from_millis(300)).await;

            // --- 1. ЧТЕНИЕ ДАННЫХ ИЗ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ ---
            let u1_mv = crate::U1_ACTUAL_MV.load(Ordering::Relaxed);
            let voltage_v = (u1_mv as f32) / 1000.0;
            let i1_mv = crate::I1_ACTUAL_MV.load(Ordering::Relaxed);
            let current_v = (i1_mv as f32) / 1000.0;
            let p1_mv = crate::P1_ACTUAL_MV.load(Ordering::Relaxed);
            let potencial_v = (p1_mv as f32) / 1000.0;
            let current_pwm = crate::PWM1_ACTUAL.load(Ordering::Relaxed) as u16;

            // Новые данные
            let bat_v = (crate::BAT_VOLTAGE_MV.load(Ordering::Relaxed) as f32) / 1000.0;
            let temp_c = (crate::TEMP_C_X10.load(Ordering::Relaxed) as f32) / 10.0;
            let door_val: u16 = if crate::DOOR_CLOSED.load(Ordering::Relaxed) {
                1
            } else {
                0
            };
            let csq = crate::GSM_SIGNAL.load(Ordering::Relaxed) as u16;

            // --- ОБНОВЛЕННЫЕ СТАТУСЫ ---
            let gprs_status: u16 = if crate::GPRS_OK.load(Ordering::Relaxed) {
                1
            } else {
                0
            };
            let server_status: u16 = if crate::GSM_OK.load(Ordering::Relaxed) {
                1
            } else {
                0
            };

            // --- 2. ОТПРАВКА FLOAT ЗНАЧЕНИЙ ---
            dwin_write_float(&mut tx, 0x5000, voltage_v).await;
            dwin_write_float(&mut tx, 0x5002, current_v).await;
            dwin_write_float(&mut tx, 0x5004, potencial_v).await;
            dwin_write_float(&mut tx, 0x5024, bat_v).await; // Напряжение батареи
            dwin_write_float(&mut tx, 0x5028, temp_c).await; // Температура

            // --- 3. ОТПРАВКА INT16 ЗНАЧЕНИЙ ---
            dwin_write_int16(&mut tx, 0x5012, current_pwm).await;
            dwin_write_int16(&mut tx, 0x5016, door_val).await; // Дверь
            dwin_write_int16(&mut tx, 0x5026, csq).await; // Антенна GSM (уровень)

            // Статусы GSM
            dwin_write_int16(&mut tx, 0x5020, gprs_status).await; // Иконка СИМ-карты / GPRS
            dwin_write_int16(&mut tx, 0x5021, server_status).await; // Иконка Сервера

            // Статичные переключатели (всегда включены на нашей плате)
            dwin_write_int16(&mut tx, 0x5017, 1).await; // Батарея (вкл)
            dwin_write_int16(&mut tx, 0x5022, 1).await; // Батарея (иконка)
            //dwin_write_int16(&mut tx, 0x5030, 1).await; // GSM переключатель (вкл)

            // Внутри tx_fut, добавь к остальным отправкам
            let kwh = f32::from_bits(crate::ENERGY_KWH.load(Ordering::Relaxed));
            let hours = crate::HOURS_WORK.load(Ordering::Relaxed);
            let gsm_on_val: u16 = if crate::GSM_ON.load(Ordering::Relaxed) {
                1
            } else {
                0
            };

            dwin_write_float(&mut tx, 0x5006, kwh).await; // Энергия[cite: 16]
            dwin_write_int32(&mut tx, 0x5008, hours).await; // Часы наработки[cite: 16]
            dwin_write_int16(&mut tx, 0x5030, gsm_on_val).await; // Состояние GSM[cite: 16]

            // --- ДОБАВЛЕННЫЕ ДАННЫЕ АВТО-РЕЖИМА ---
            let auto_mode_val: u16 = if crate::AUTO_MODE.load(Ordering::Relaxed) {
                1
            } else {
                0
            };
            // Всегда отдаем экрану чистое положительное число вольтах (Float)
            let target_mv = crate::AUTO_TARGET_MV.load(Ordering::Relaxed).abs();
            let target_v_display = (target_mv as f32) / 1000.0;

            dwin_write_int16(&mut tx, 0x5031, auto_mode_val).await;
            dwin_write_float(&mut tx, 0x5032, target_v_display).await;

            let limit_val: u16 = if crate::LIMIT_10A_EN.load(Ordering::Relaxed) {
                1
            } else {
                0
            };
            let vi_mode_val: u16 = if crate::V_I_MODE_EN.load(Ordering::Relaxed) {
                1
            } else {
                0
            };

            dwin_write_int16(&mut tx, 0x5038, limit_val).await;
            dwin_write_int16(&mut tx, 0x5039, vi_mode_val).await;

            // ... рядом с отправкой энергии или часов (например, строка ~200) ...

            let pol_pot_mv = crate::POLARIZATION_POTENTIAL_MV.load(Ordering::Relaxed);
            let pol_pot_v = (pol_pot_mv as f32) / 1000.0;

            // Отправляем защитный потенциал как Float
            dwin_write_float(&mut tx, 0x5010, pol_pot_v).await;

            // Если для Bluetooth есть свой адрес в DGUS (например, 0x5032), добавь:
            // dwin_write_int16(&mut tx, 0x5032, 1).await;
        }
    };

    join(rx_fut, tx_fut).await;
}
