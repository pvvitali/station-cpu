use core::str;
use defmt::{error, info};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{Peri, bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write;
use heapless::String;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
    DMA2_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH7>;
});

fn crc_8(data: &[u8]) -> u8 {
    let mut crc = 0xFF;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x01) != 0 {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

macro_rules! send_at {
    ($uart:expr, $rx_buf:expr, $cmd:expr, $expected:expr, $wait_ms:expr) => {{
        // Вычитываем буфер досуха (пока не наступит таймаут)
        while let Ok(_) = with_timeout(
            Duration::from_millis(50),
            $uart.read_until_idle(&mut $rx_buf),
        )
        .await
        {}

        let mut success = false;
        if $uart.write_all($cmd).await.is_ok() {
            let start = embassy_time::Instant::now();
            while start.elapsed().as_millis() < $wait_ms as u64 {
                let elapsed = start.elapsed().as_millis() as u64;
                let remaining = if $wait_ms as u64 > elapsed { $wait_ms as u64 - elapsed } else { 10 };

                if let Ok(Ok(len)) = with_timeout(
                    Duration::from_millis(remaining),
                    $uart.read_until_idle(&mut $rx_buf),
                )
                .await
                {
                    if let Ok(resp) = str::from_utf8(&$rx_buf[..len]) {
                        if resp.contains($expected) {
                            success = true;
                            break;
                        }
                        if resp.contains("ERROR") {
                            error!("GSM REJECT: {}", resp.trim());
                            break;
                        }
                    }
                }
            }
        }
        success
    }};
}

#[embassy_executor::task]
pub async fn gsm_task(
    usart1: Peri<'static, peripherals::USART1>,
    rx_pin: Peri<'static, peripherals::PA10>,
    tx_pin: Peri<'static, peripherals::PA9>,
    rx_dma: Peri<'static, peripherals::DMA2_CH2>,
    tx_dma: Peri<'static, peripherals::DMA2_CH7>,
    mut pwrkey: Output<'static>,
    mut reset: Output<'static>,
    mut dtr: Output<'static>,
    status: Input<'static>,
    _ring: Input<'static>,
) {
    let mut config = Config::default();
    config.baudrate = 115200;

    let mut uart = Uart::new(usart1, rx_pin, tx_pin, tx_dma, rx_dma, Irqs, config).unwrap();
    let mut rx_buf = [0u8; 1024];

    info!("GSM: Запуск...");
    pwrkey.set_low();
    reset.set_low();
    dtr.set_low();

    if status.is_low() {
        info!("GSM: Модем выключен. Запускаем (PWRKEY)...");
        pwrkey.set_high();
        Timer::after_millis(1000).await;
        pwrkey.set_low();

        let mut timeout = 0;
        while status.is_low() && timeout < 50 {
            Timer::after_millis(100).await;
            timeout += 1;
        }
        if status.is_low() {
            error!("GSM: Ошибка запуска! STATUS не поднялся.");
        } else {
            info!("STATUS is high! OK!");
        }
    } else {
        info!("STATUS is high in on module");
    }
    Timer::after_secs(3).await;

    let _ = with_timeout(
        Duration::from_millis(500),
        uart.read_until_idle(&mut rx_buf),
    )
    .await;

    loop {
        if send_at!(uart, rx_buf, b"AT\r\n", "OK", 1000) {
            break;
        }
        Timer::after_millis(500).await;
    }
    send_at!(uart, rx_buf, b"ATE0\r\n", "OK", 1000);
    info!("GSM: Связь AT установлена.");

    loop {
        info!("GSM: Регистрация в сети и настройка...");

        loop {
            let _ =
                with_timeout(Duration::from_millis(50), uart.read_until_idle(&mut rx_buf)).await;
            let _ = uart.write_all(b"AT+CPIN?\r\n").await;
            let _ = with_timeout(
                Duration::from_millis(1000),
                uart.read_until_idle(&mut rx_buf),
            )
            .await;

            let _ = uart.write_all(b"AT+CEREG?\r\n").await;
            if let Ok(Ok(len)) = with_timeout(
                Duration::from_millis(1000),
                uart.read_until_idle(&mut rx_buf),
            )
            .await
            {
                if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                    let text = resp.trim();
                    if text.contains("+CEREG: 0,1\r")
                        || text.contains("+CEREG: 0,5\r")
                        || text.contains("+CEREG: 1,1\r")
                        || text.contains("+CEREG: 1,5\r")
                    {
                        break;
                    }
                }
            }

            let _ = uart.write_all(b"AT+CREG?\r\n").await;
            if let Ok(Ok(len)) = with_timeout(
                Duration::from_millis(1000),
                uart.read_until_idle(&mut rx_buf),
            )
            .await
            {
                if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                    let text = resp.trim();
                    if text.contains("+CREG: 0,1\r") || text.contains("+CREG: 0,5\r") {
                        break;
                    }
                }
            }
            Timer::after_secs(3).await;
        }

        info!("GSM: Регистрация успешна! Настраиваем APN...");

        send_at!(uart, rx_buf, b"AT+CGDCONT=1,\"IP\",\"www\"\r\n", "OK", 2000);
        send_at!(uart, rx_buf, b"AT+CGACT=1,1\r\n", "OK", 5000);

        // ПРИНУДИТЕЛЬНАЯ ОЧИСТКА: Закрываем старые соединения перед открытием новых
        info!("GSM: Очистка старых сессий...");
        let _ = send_at!(uart, rx_buf, b"AT+CIPCLOSE=0\r\n", "OK", 1500);
        let _ = send_at!(uart, rx_buf, b"AT+NETCLOSE\r\n", "OK", 1500);

        let _ = send_at!(uart, rx_buf, b"AT+NETOPEN\r\n", "OK", 5000);
        Timer::after_secs(1).await;

        info!("GSM: Подключение к TCP серверу scz.pge.md:16992...");

        if send_at!(
            uart,
            rx_buf,
            b"AT+CIPOPEN=0,\"TCP\",\"scz.pge.md\",16992\r\n",
            "+CIPOPEN: 0,0",
            15000
        ) {
            info!("GSM: Успешно подключено к серверу!");

            loop {
                // Читаем реальные данные с датчиков (милливольты)
                let u1_mv = crate::U1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let i1_mv = crate::I1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let p1_mv = crate::P1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);

                // Форматируем для сервера (напряжение и ток /100, потенциал /10)
                let u_str = u1_mv / 100;
                let i_str = i1_mv / 100;
                let p_str = p1_mv / 10;

                let mut payload = String::<256>::new();
                core::fmt::write(
                    &mut payload,
                    format_args!(
                        "{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|",
                        "10000600",
                        i_str,
                        u_str,
                        p_str,
                        "0",
                        "24",
                        "0",
                        "0",
                        "0",
                        "0",
                        "0",
                        "0",
                        "0",
                        "0",
                        "0"
                    ),
                )
                .unwrap();

                let crc = crc_8(payload.as_bytes());
                core::fmt::write(&mut payload, format_args!("{};", crc)).unwrap();

                // Формируем команду CIPSEND с указанием точной длины пакета
                let mut cmd = String::<32>::new();
                core::fmt::write(&mut cmd, format_args!("AT+CIPSEND=0,{}\r\n", payload.len()))
                    .unwrap();

                if send_at!(uart, rx_buf, cmd.as_bytes(), ">", 2000) {
                    info!("GSM: Отправка телеметрии ({} байт)...", payload.len());

                    if send_at!(uart, rx_buf, payload.as_bytes(), "OK", 10000) {
                        info!("GSM: Пакет успешно отправлен!");
                    } else {
                        error!("GSM: Ошибка передачи данных.");
                        break;
                    }
                } else {
                    error!("GSM: Сервер разорвал соединение или модем не готов.");
                    break;
                }

                info!("GSM: Ожидание 60 сек. Слушаем ответы сервера...");

                let wait_start = embassy_time::Instant::now();
                while wait_start.elapsed().as_secs() < 60 {
                    if let Ok(Ok(len)) = with_timeout(
                        Duration::from_millis(500),
                        uart.read_until_idle(&mut rx_buf),
                    )
                    .await
                    {
                        if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                            let text = resp.trim();
                            if !text.is_empty() {
                                // Выводим команды от сервера в консоль
                                info!("GSM <- SERVER MSG: {}", text);
                            }
                        }
                    }
                }
            }
        } else {
            error!("GSM: Ошибка подключения к TCP. Повтор через 10 сек...");
        }

        send_at!(uart, rx_buf, b"AT+CIPCLOSE=0\r\n", "OK", 2000);
        Timer::after_secs(10).await;
    }
}
