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

#[derive(Debug, defmt::Format)]
pub enum ServerCommand {
    Heartbeat(heapless::String<10>), // $1|tx_id|
    SetPeriod {
        tx_id: heapless::String<10>,
        period_ms: u32,
    }, // $3|tx_id|period|
    StepUp1V(heapless::String<10>),  // $5
    StepDown1V(heapless::String<10>), // $6
    StepUp10V(heapless::String<10>), // $7
    StepDown10V(heapless::String<10>), // $8
    SetAuto {
        tx_id: heapless::String<10>,
        target_u_mv: u32,
    }, // $9|tx_id|target|
    DisableAuto(heapless::String<10>), // $10
    PowerOn(heapless::String<10>),   // $11
    PowerOff(heapless::String<10>),  // $12
    StepUp0_1V(heapless::String<10>), // $13
    StepDown0_1V(heapless::String<10>), // $14
    UpdateFlash(heapless::String<10>), // $15
    Unknown,
}

// Новая функция для безопасного парсинга
fn parse_server_command(msg: &str) -> ServerCommand {
    // 1. Ищем начало пакета
    let start_idx = match msg.find('$') {
        Some(idx) => idx,
        None => return ServerCommand::Unknown,
    };

    // 2. Ищем конец пакета (точку с запятой перед CRC, либо просто конец команды)
    let end_idx = msg.find(';').unwrap_or(msg.len());
    let payload = &msg[start_idx + 1..end_idx]; // Вырезаем от $ до ;

    // 3. Разбиваем строку по разделителю '|'
    // Используем heapless::Vec, чтобы не выделять память динамически
    let parts: heapless::Vec<&str, 8> = payload.split('|').collect();

    if parts.len() < 2 {
        return ServerCommand::Unknown; // Как минимум должна быть команда и tx_id
    }

    let cmd_id = parts[0];

    let mut tx_id = heapless::String::<10>::new();
    let _ = tx_id.push_str(parts[1]);

    match cmd_id {
        "1" => ServerCommand::Heartbeat(tx_id),
        "3" if parts.len() >= 3 => {
            // Парсим строку в u32 безопасно
            if let Ok(period) = parts[2].parse::<u32>() {
                ServerCommand::SetPeriod {
                    tx_id,
                    period_ms: period,
                }
            } else {
                ServerCommand::Unknown
            }
        }
        "5" => ServerCommand::StepUp1V(tx_id),
        "6" => ServerCommand::StepDown1V(tx_id),
        "7" => ServerCommand::StepUp10V(tx_id),
        "8" => ServerCommand::StepDown10V(tx_id),
        "9" if parts.len() >= 3 => {
            if let Ok(target) = parts[2].parse::<u32>() {
                ServerCommand::SetAuto {
                    tx_id,
                    target_u_mv: target,
                }
            } else {
                ServerCommand::Unknown
            }
        }
        "10" => ServerCommand::DisableAuto(tx_id),
        "11" => ServerCommand::PowerOn(tx_id),
        "12" => ServerCommand::PowerOff(tx_id),
        "13" => ServerCommand::StepUp0_1V(tx_id),
        "14" => ServerCommand::StepDown0_1V(tx_id),
        "15" => ServerCommand::UpdateFlash(tx_id),
        _ => ServerCommand::Unknown,
    }
}

// Функция для отправки ответа серверу
macro_rules! send_server_ok {
    ($uart:expr, $rx_buf:expr, $cmd_id:expr, $tx_id:expr) => {{
        let mut payload = heapless::String::<64>::new();
        core::fmt::write(&mut payload, format_args!("OK|{}|{}|", $cmd_id, $tx_id)).unwrap();

        let crc = crc_8(payload.as_bytes());
        core::fmt::write(&mut payload, format_args!("{};\x1A", crc)).unwrap();

        let mut cmd = heapless::String::<32>::new();
        core::fmt::write(&mut cmd, format_args!("AT+CIPSEND=0,{}\r\n", payload.len())).unwrap();

        // Просим модем открыть поток данных
        if send_at!($uart, $rx_buf, cmd.as_bytes(), ">", 2000) {
            // Отправляем сам пакет
            if send_at!($uart, $rx_buf, payload.as_bytes(), "OK", 3000) {
                info!(
                    "GSM: Ответ серверу OK|{}|{}| успешно отправлен!",
                    $cmd_id, $tx_id
                );
            } else {
                error!("GSM: Ошибка отправки ответа серверу.");
            }
        }
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
                // Читаем реальные данные с датчиков
                let u1_mv = crate::U1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let i1_mv = crate::I1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let p1_mv = crate::P1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let bat_mv = crate::BAT_VOLTAGE_MV.load(core::sync::atomic::Ordering::Relaxed);
                let temp_x10 = crate::TEMP_C_X10.load(core::sync::atomic::Ordering::Relaxed);
                let csq = crate::GSM_SIGNAL.load(core::sync::atomic::Ordering::Relaxed);

                let is_door_closed = crate::DOOR_CLOSED.load(core::sync::atomic::Ordering::Relaxed);
                let is_220v_ok = if u1_mv > 10_000 { 1 } else { 0 }; // Условный флаг 220В

                // В C-коде к напряжению и току прибавлялось 0.05 перед умножением (округление)
                // Мы сделаем целочисленное деление, так как у нас уже милливольты
                let u_str = u1_mv / 100;
                let i_str = i1_mv / 100;
                let p_str = p1_mv / 10;
                let bat_str = bat_mv / 100;

                // Энергия в кВт*ч (нужно умножить на 10 для формата сервера)
                let energy_kwh =
                    f32::from_bits(crate::ENERGY_KWH.load(core::sync::atomic::Ordering::Relaxed));
                let energy_str = (energy_kwh * 10.0) as u32;

                let door_str = if is_door_closed { "0" } else { "1" }; // 0-закрыто, 1-открыто

                let mut payload = String::<256>::new();
                core::fmt::write(
                    &mut payload,
                    format_args!(
                        "{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|",
                        "10001005",    // ID станции
                        i_str,         // i
                        u_str,         // u
                        p_str,         // p
                        bat_str,       // u_bat
                        csq,           // sq (сигнал)
                        is_220v_ok,    // u220
                        door_str,      // door
                        temp_x10 / 10, // temperature (сервер ждет целые градусы)
                        energy_str,    // power (энергия * 10)
                        "0",           // i_max
                        "0",           // t_max
                        "6",           // period_transfer (в десятках секунд, 6 = 60 сек)
                        "0",           // auto_on_off
                        "0"            // auto_value
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
                                // Выводим сырые данные
                                info!("GSM <- SERVER MSG: {}", text);

                                // Парсим команду
                                let cmd = parse_server_command(text);
                                match cmd {
                                    ServerCommand::Heartbeat(tx_id) => {
                                        info!(
                                            "Получен Heartbeat (tx: {}). Отвечаем...",
                                            tx_id.as_str()
                                        );
                                        send_server_ok!(uart, rx_buf, "1", tx_id.as_str());
                                    }
                                    ServerCommand::StepUp10V(tx_id) => {
                                        // Вызываем .as_str()
                                        info!("Получена команда +10V (tx: {})", tx_id.as_str());
                                        // TODO: Отправить команду в доводчик напряжения
                                    }
                                    ServerCommand::PowerOff(tx_id) => {
                                        // Вызываем .as_str()
                                        info!(
                                            "Получена команда Выкл. питания (tx: {})",
                                            tx_id.as_str()
                                        );
                                        // TODO: Сбросить ШИМ в 0
                                    }
                                    ServerCommand::Unknown => {
                                        // Игнорируем нераспознанные строки
                                    }
                                    _ => {
                                        // Выводим саму структуру (теперь это работает благодаря defmt::Format)
                                        info!("Получена команда: {:?}", cmd);
                                    }
                                }
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
