use defmt::{info, unwrap};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{BufferedUart, Config};
use embassy_stm32::{Peri, bind_interrupts, peripherals, usart};
use embassy_time::Timer;
use embedded_io_async::{BufRead, Write};

// Привязываем прерывания к USART3
bind_interrupts!(struct Irqs {
    USART3 => usart::BufferedInterruptHandler<peripherals::USART3>;
});

#[embassy_executor::task]
pub async fn rs485_task(
    usart3: Peri<'static, peripherals::USART3>,
    rx_pin: Peri<'static, peripherals::PC11>,
    tx_pin: Peri<'static, peripherals::PC10>,
    rw_pin_peri: Peri<'static, peripherals::PC12>,
) {
    let mut tx_buf = [0u8; 128];
    let mut rx_buf = [0u8; 128];

    // Настраиваем скорость 2400 бод
    let mut config = Config::default();
    config.baudrate = 2400;

    let mut rs485_uart = BufferedUart::new(
        usart3,
        rx_pin,
        tx_pin,
        &mut tx_buf,
        &mut rx_buf,
        Irqs,
        config,
    )
    .unwrap();

    // Инициализируем пин управления (RE/DE).
    // По умолчанию Level::Low — режим ПРИЕМА (слушаем линию).
    let mut rw_pin = Output::new(rw_pin_peri, Level::Low, Speed::High);

    info!("RS485 task started (PC11=RX, PC10=TX, PC12=RW) at 2400 baud");

    let target_seq = b"$?;";
    let mut match_state = 0;

    loop {
        let mut should_reply = false; // Флаг для ответа

        // 1. Блокируем UART для чтения
        let buf = rs485_uart.fill_buf().await.unwrap();
        let n = buf.len();

        // 2. Ищем команду в полученном куске данных
        for &byte in buf {
            if byte == target_seq[match_state] {
                match_state += 1;

                if match_state == target_seq.len() {
                    match_state = 0;
                    should_reply = true; // Нашли! Ставим галочку, что нужно ответить
                }
            } else if byte == target_seq[0] {
                match_state = 1;
            } else {
                match_state = 0;
            }
        }

        // 3. Сообщаем UART, что мы прочитали данные. Снимаем блокировку!
        rs485_uart.consume(n);

        // 4. Теперь UART свободен, можем переключать пин и отправлять ответ
        if should_reply {
            info!("RS485 RX: $?;");

            rw_pin.set_high(); // Режим ПЕРЕДАЧИ

            unwrap!(rs485_uart.write_all(b"$20|11|2|0|0|10|0|1|0|;\r\n").await);
            unwrap!(rs485_uart.flush().await);

            //delay for reseiver
            Timer::after_millis(500).await;

            unwrap!(rs485_uart.write_all(b"$20|11|2|0|0|10|0|1|0|;\r\n").await);
            unwrap!(rs485_uart.flush().await);

            rw_pin.set_low(); // Режим ПРИЕМА

            info!("RS485 TX: $20|11|2|0|0|10|0|1|0|;");
        }
    }
}
