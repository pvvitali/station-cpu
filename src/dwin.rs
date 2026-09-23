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
                                        let data = ((pkt[7] as u16) << 8) | (pkt[8] as u16);

                                        match addr {
                                            0x5012 => info!("-> Изменение ШИМ: {}", data),
                                            0x5017 => info!("-> Аккумулятор: {}", data),
                                            0x5030 => info!("-> SIM800: {}", data),
                                            _ => {}
                                        }
                                    }
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

            let u1_mv = crate::U1_ACTUAL_MV.load(Ordering::Relaxed);
            let voltage_v = (u1_mv as f32) / 1000.0;
            let i1_mv = crate::I1_ACTUAL_MV.load(Ordering::Relaxed);
            let current_v = (i1_mv as f32) / 1000.0;
            let p1_mv = crate::P1_ACTUAL_MV.load(Ordering::Relaxed);
            let potencial_v = (p1_mv as f32) / 1000.0;

            dwin_write_float(&mut tx, 0x5000, voltage_v).await;
            dwin_write_float(&mut tx, 0x5002, current_v).await;
            dwin_write_float(&mut tx, 0x5004, potencial_v).await;
        }
    };

    join(rx_fut, tx_fut).await;
}
