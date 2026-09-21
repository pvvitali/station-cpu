use defmt::{info, unwrap};
use embassy_stm32::usart::{BufferedUart, Config};
use embassy_stm32::{Peri, bind_interrupts, peripherals, usart};
use embedded_io_async::{BufRead, Write};

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

#[embassy_executor::task]
pub async fn bluetooth_task(
    usart2: Peri<'static, peripherals::USART2>,
    rx_pin: Peri<'static, peripherals::PA3>,
    tx_pin: Peri<'static, peripherals::PA2>,
) {
    let mut tx_buf = [0u8; 256];
    let mut rx_buf = [0u8; 256];

    let config = Config::default();

    // Идеальный порядок для Embassy 0.6.0
    let mut bt_uart = BufferedUart::new(
        usart2,
        rx_pin,      // 2. Сначала RX
        tx_pin,      // 3. Затем TX
        &mut tx_buf, // 4. Буфер передачи
        &mut rx_buf, // 5. Буфер приема
        Irqs,        // 6. Обработчик прерываний
        config,      // 7. Конфигурация
    )
    .unwrap();

    info!("Bluetooth task started (PA3=RX, PA2=TX)");

    loop {
        let buf = bt_uart.fill_buf().await.unwrap();
        let n = buf.len();

        if n > 0 {
            match core::str::from_utf8(buf) {
                Ok(s) => info!("BT RX: {}", s),
                Err(_) => info!("BT RX (raw bytes): {=[u8]:x}", buf),
            }

            bt_uart.consume(n);
            unwrap!(bt_uart.write_all(b"AAA\r\n").await);
            info!("BT TX: AAA");
        }
    }
}
