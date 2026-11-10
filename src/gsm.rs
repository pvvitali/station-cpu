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
        target_u_mv: i32,
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
            if let Ok(target) = parts[2].parse::<i32>() {
                ServerCommand::SetAuto {
                    tx_id,
                    target_u_mv: target * 10, // Переводим -125 (сотые) в -1250 мВ!
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
        core::fmt::write(&mut payload, format_args!("{};", crc)).unwrap();

        let mut cmd = heapless::String::<32>::new();
        core::fmt::write(&mut cmd, format_args!("AT+CIPSEND=0,{}\r\n", payload.len())).unwrap();

        // Просим модем открыть поток данных
        if send_at!($uart, $rx_buf, cmd.as_bytes(), ">", 2000) {
            // Отправляем сам пакет
            if send_at!($uart, $rx_buf, payload.as_bytes(), "OK", 3000) {
                info!(
                    "GSM: Ответ серверу OK|{}|{}|src; успешно отправлен!",
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
        // --- СБРОС ФЛАГОВ ПРИ ПОТЕРИ СВЯЗИ И ПЕРЕЗАПУСКЕ ---
        crate::GPRS_OK.store(false, core::sync::atomic::Ordering::Relaxed);
        crate::GSM_OK.store(false, core::sync::atomic::Ordering::Relaxed);

        if send_at!(uart, rx_buf, b"AT\r\n", "OK", 1000) {
            break;
        }
        Timer::after_millis(500).await;
    }
    send_at!(uart, rx_buf, b"ATE0\r\n", "OK", 1000);
    info!("GSM: Связь AT установлена.");

    loop {
        // --- СБРОС ФЛАГОВ ПЕРЕД НОВОЙ ПОПЫТКОЙ РЕГИСТРАЦИИ ---
        crate::GSM_OK.store(false, core::sync::atomic::Ordering::Relaxed);

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

        // --- УСТАНОВКА ФЛАГА GPRS ПОСЛЕ УСПЕШНОЙ АКТИВАЦИИ APN ---
        crate::GPRS_OK.store(true, core::sync::atomic::Ordering::Relaxed);

        // ПРИНУДИТЕЛЬНАЯ ОЧИСТКА: Закрываем старые соединения перед открытием новых
        info!("GSM: Очистка старых сессий...");
        let _ = send_at!(uart, rx_buf, b"AT+CIPCLOSE=0\r\n", "OK", 1500);
        let _ = send_at!(uart, rx_buf, b"AT+NETCLOSE\r\n", "OK", 1500);

        let _ = send_at!(uart, rx_buf, b"AT+NETOPEN\r\n", "OK", 5000);
        Timer::after_secs(1).await;

        // === НОВЫЙ БЛОК: Запрашиваем сигнал перед первой отправкой ===
        let _ = uart.write_all(b"AT+CSQ\r\n").await;
        if let Ok(Ok(len)) = with_timeout(
            Duration::from_millis(500),
            uart.read_until_idle(&mut rx_buf),
        )
        .await
        {
            if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                if let Some(csq_idx) = resp.find("+CSQ: ") {
                    let tail = &resp[csq_idx + 6..];
                    if let Some(comma_idx) = tail.find(',') {
                        if let Ok(csq_val) = tail[..comma_idx].trim().parse::<u32>() {
                            let valid_csq = if csq_val == 99 { 0 } else { csq_val };
                            crate::GSM_SIGNAL
                                .store(valid_csq, core::sync::atomic::Ordering::Relaxed);
                        }
                    }
                }
            }
        }
        // =============================================================

        info!("GSM: Подключение к TCP серверу scz.pge.md:16992...");

        if send_at!(
            uart,
            rx_buf,
            b"AT+CIPOPEN=0,\"TCP\",\"scz.pge.md\",16992\r\n",
            "+CIPOPEN: 0,0",
            15000
        ) {
            info!("GSM: Успешно подключено к серверу!");

            // --- УСТАНОВКА ФЛАГА СЕРВЕРА ПРИ УСПЕШНОМ TCP-СОЕДИНЕНИИ ---
            crate::GSM_OK.store(true, core::sync::atomic::Ordering::Relaxed);

            loop {
                // Читаем реальные данные с датчиков
                let u1_mv = crate::U1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let i1_mv = crate::I1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let p1_mv = crate::P1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let bat_mv = crate::BAT_VOLTAGE_MV.load(core::sync::atomic::Ordering::Relaxed);
                let temp_x10 = crate::TEMP_C_X10.load(core::sync::atomic::Ordering::Relaxed);
                let csq_raw = crate::GSM_SIGNAL.load(core::sync::atomic::Ordering::Relaxed);

                // 0 - HardwareId
                let id_str = "10000601";

                // 1 - Выходной ток * 10 (мА / 100) -> 30.2A = 302
                let i_str = i1_mv / 100;

                // 2 - Выходное напряжение * 10 (мВ / 100) -> -21.5V = -215
                let u_str = u1_mv / 100;

                // 3 - Защитный потенциал * 100 (мВ / 10) -> 8.3V = 830
                let p_str = p1_mv / 10;

                // 4 - Напряжение батареи * 10 (мВ / 100) -> 4.2V = 42
                let bat_str = bat_mv / 100;

                // сырые 0..31
                let gsm_percent = csq_raw;

                // 6 - Наличие 220V (1 - есть, 0 - нет)
                let is_on_battery = crate::POWER_SOURCE.load(core::sync::atomic::Ordering::Relaxed);
                let is_220v_ok = if !is_on_battery { "1" } else { "0" };

                // 7 - Дверь (0 - закрыта, 1 - открыта)
                let is_door_closed = crate::DOOR_CLOSED.load(core::sync::atomic::Ordering::Relaxed);
                let door_str = if is_door_closed { "0" } else { "1" };

                // 8 - Температура (целые градусы)
                let temp_c = temp_x10 / 10;

                // 9 - Энергия кВт*ч (целое число [int], без умножения на 10)
                let energy_kwh =
                    f32::from_bits(crate::ENERGY_KWH.load(core::sync::atomic::Ordering::Relaxed))
                        as u32;

                // 10, 11 - Аварии по току и температуре (0 - ок)
                let alarm_i = "0";
                let alarm_t = "0";

                // 12 - Интервал передачи (сервер ждет в десятках секунд)
                let period_secs =
                    crate::TELEMETRY_PERIOD_SECS.load(core::sync::atomic::Ordering::Relaxed);
                let period_for_server = period_secs / 10;

                // 13 - Авто режим (0/1)
                let is_auto = crate::AUTO_MODE.load(core::sync::atomic::Ordering::Relaxed);
                let auto_mode = if is_auto { "1" } else { "0" };

                // 14 - Целевой потенциал (делим на 10, чтобы из -1250 мВ получить -125 для сервера)
                let target_mv = crate::AUTO_TARGET_MV.load(core::sync::atomic::Ordering::Relaxed);
                let auto_value = target_mv / 10;

                let mut payload = heapless::String::<256>::new();
                core::fmt::write(
                    &mut payload,
                    format_args!(
                        "{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|",
                        id_str,            // 0
                        i_str,             // 1
                        u_str,             // 2
                        p_str,             // 3
                        bat_str,           // 4
                        gsm_percent,       // 5
                        is_220v_ok,        // 6
                        door_str,          // 7
                        temp_c,            // 8
                        energy_kwh,        // 9
                        alarm_i,           // 10
                        alarm_t,           // 11
                        period_for_server, // 12
                        auto_mode,         // 13
                        auto_value         // 14
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
                        // --- ДОБАВИТЬ СБРОС ЗДЕСЬ ---
                        crate::GSM_OK.store(false, core::sync::atomic::Ordering::Relaxed);
                        break;
                    }
                } else {
                    error!("GSM: Сервер разорвал соединение или модем не готов.");
                    crate::GSM_OK.store(false, core::sync::atomic::Ordering::Relaxed);
                    break;
                }

                info!("GSM: Ожидание 60 сек. Слушаем ответы сервера...");

                let wait_start = embassy_time::Instant::now();
                let mut last_csq_check = embassy_time::Instant::now();

                let mut current_period =
                    crate::TELEMETRY_PERIOD_SECS.load(core::sync::atomic::Ordering::Relaxed) as u64;
                // Защита от "DDOS-атаки": если сервер пришлет слишком маленький интервал, ограничиваем минимум 10 секундами
                if current_period < 10 {
                    current_period = 10;
                }

                // Спим заданное количество секунд
                while wait_start.elapsed().as_secs() < current_period {
                    // --- ПРОВЕРЯЕМ СИГНАЛ ОТ ДОВОДЧИКА ---
                    if crate::FORCE_TELEMETRY_UPDATE.load(core::sync::atomic::Ordering::Relaxed) {
                        crate::FORCE_TELEMETRY_UPDATE
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        info!("GSM: Доводчик завершил работу. Внеочередная отправка телеметрии!");
                        break; // Выходим из ожидания, сразу начнется новый виток с отправкой данных!
                    }

                    // --- ОПРОС АНТЕННЫ КАЖДЫЕ 5 СЕКУНД ---
                    if last_csq_check.elapsed().as_secs() >= 5 {
                        let _ = uart.write_all(b"AT+CSQ\r\n").await;
                        last_csq_check = embassy_time::Instant::now();
                    }

                    if let Ok(Ok(len)) = with_timeout(
                        Duration::from_millis(500),
                        uart.read_until_idle(&mut rx_buf),
                    )
                    .await
                    {
                        if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                            let text = resp.trim();
                            if !text.is_empty() {
                                // 1. Парсим ответ об уровне сигнала (+CSQ: 15,0)
                                if let Some(csq_idx) = text.find("+CSQ: ") {
                                    let tail = &text[csq_idx + 6..];
                                    if let Some(comma_idx) = tail.find(',') {
                                        if let Ok(csq_val) = tail[..comma_idx].trim().parse::<u32>()
                                        {
                                            // В SIM800 значение 99 означает "нет сигнала"
                                            let valid_csq = if csq_val == 99 { 0 } else { csq_val };
                                            crate::GSM_SIGNAL.store(
                                                valid_csq,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                        }
                                    }
                                }

                                // 2. Парсим команды от сервера (содержат $ или +IPD)
                                if text.contains("+IPD") || text.contains('$') {
                                    info!("GSM <- SERVER MSG: {}", text);
                                    let cmd = parse_server_command(text);
                                    match cmd {
                                        ServerCommand::Heartbeat(tx_id) => {
                                            info!(
                                                "GSM: Запрос обновления данных (tx: {})",
                                                tx_id.as_str()
                                            );

                                            // 1. Отвечаем серверу, что команда принята
                                            send_server_ok!(uart, rx_buf, "1", tx_id.as_str());

                                            // 2. Даем небольшую паузу, чтобы сервер успел переварить #OK
                                            Timer::after_millis(100).await;

                                            // 3. ПРЕРЫВАЕМ 60-СЕКУНДНОЕ ОЖИДАНИЕ!
                                            // Программа выйдет из `while` и сразу начнет новый виток `loop`,
                                            // соберет свежую телеметрию и отправит её.
                                            break;
                                        }
                                        ServerCommand::StepUp10V(tx_id) => {
                                            info!("GSM: Команда +10V (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            let _ = crate::ADJUST_CHANNEL
                                                .try_send(crate::AdjustCmd::Up10V);
                                            send_server_ok!(uart, rx_buf, "7", tx_id.as_str());
                                        }
                                        ServerCommand::StepDown10V(tx_id) => {
                                            info!("GSM: Команда -10V (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            let _ = crate::ADJUST_CHANNEL
                                                .try_send(crate::AdjustCmd::Down10V);
                                            send_server_ok!(uart, rx_buf, "8", tx_id.as_str());
                                        }
                                        ServerCommand::StepUp1V(tx_id) => {
                                            info!("GSM: Команда +1V (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            let _ = crate::ADJUST_CHANNEL
                                                .try_send(crate::AdjustCmd::Up1V);
                                            send_server_ok!(uart, rx_buf, "5", tx_id.as_str());
                                        }
                                        ServerCommand::StepDown1V(tx_id) => {
                                            info!("GSM: Команда -1V (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            let _ = crate::ADJUST_CHANNEL
                                                .try_send(crate::AdjustCmd::Down1V);
                                            send_server_ok!(uart, rx_buf, "6", tx_id.as_str());
                                        }
                                        ServerCommand::StepUp0_1V(tx_id) => {
                                            info!("GSM: Команда +0.1V (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            let _ = crate::ADJUST_CHANNEL
                                                .try_send(crate::AdjustCmd::Up0_1V);
                                            send_server_ok!(uart, rx_buf, "13", tx_id.as_str());
                                        }
                                        ServerCommand::StepDown0_1V(tx_id) => {
                                            info!("GSM: Команда -0.1V (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            let _ = crate::ADJUST_CHANNEL
                                                .try_send(crate::AdjustCmd::Down0_1V);
                                            send_server_ok!(uart, rx_buf, "14", tx_id.as_str());
                                        }
                                        ServerCommand::PowerOff(tx_id) => {
                                            info!("GSM: Команда Выкл. (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );

                                            // Шлем 0 и приклеиваем бит 31 (1 << 31), чтобы пропустить запись
                                            let _ = crate::PWM_CHANNEL.try_send(0 | (1 << 31));
                                            send_server_ok!(uart, rx_buf, "12", tx_id.as_str());

                                            // Даем конденсаторам 500 мс на разряд и высылаем свежую телеметрию
                                            Timer::after_millis(500).await;
                                            crate::FORCE_TELEMETRY_UPDATE
                                                .store(true, core::sync::atomic::Ordering::Relaxed);
                                        }
                                        ServerCommand::PowerOn(tx_id) => {
                                            info!("GSM: Команда Вкл. (tx: {})", tx_id.as_str());
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );

                                            // Читаем сохраненное значение напрямую из сектора флеш-памяти
                                            let flash_addr: u32 = 0x080E_0000;
                                            let saved_config = unsafe {
                                                &*(flash_addr as *const crate::SavedConfig)
                                            };
                                            let restore_pwm = if saved_config.magic == 0xAA55AA55 {
                                                saved_config.pwm1_logical
                                            } else {
                                                0
                                            };

                                            // Восстанавливаем ШИМ, приклеиваем бит 31 (в памяти и так уже лежит это число, не затираем ресурс флешки)
                                            let _ = crate::PWM_CHANNEL
                                                .try_send(restore_pwm | (1 << 31));
                                            send_server_ok!(uart, rx_buf, "11", tx_id.as_str());

                                            // Ждем стабилизации напряжения и высылаем телеметрию
                                            Timer::after_millis(500).await;
                                            crate::FORCE_TELEMETRY_UPDATE
                                                .store(true, core::sync::atomic::Ordering::Relaxed);
                                        }
                                        ServerCommand::SetPeriod { tx_id, period_ms } => {
                                            info!(
                                                "GSM: Изменение интервала передачи (tx: {}, period: {} сек)",
                                                tx_id.as_str(),
                                                period_ms * 10
                                            );

                                            // 1. Сохраняем новый интервал
                                            crate::TELEMETRY_PERIOD_SECS.store(
                                                period_ms * 10,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );

                                            // --- ДОБАВЛЯЕМ ЗАПРОС НА СОХРАНЕНИЕ ---
                                            crate::SAVE_FLASH_REQUEST
                                                .store(true, core::sync::atomic::Ordering::Relaxed);

                                            // 2. Отвечаем серверу
                                            send_server_ok!(uart, rx_buf, "3", tx_id.as_str());
                                            Timer::after_millis(200).await;

                                            // 3. Прерываем текущее ожидание, чтобы станция сразу отправила
                                            // свежий пакет, в котором 12-е поле уже будет содержать новый интервал.
                                            break;
                                        }
                                        ServerCommand::SetAuto { tx_id, target_u_mv } => {
                                            info!(
                                                "GSM: Включен АВТО режим. Цель: {} мВ (tx: {})",
                                                target_u_mv,
                                                tx_id.as_str()
                                            );
                                            crate::AUTO_TARGET_MV.store(
                                                target_u_mv,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            crate::AUTO_MODE
                                                .store(true, core::sync::atomic::Ordering::Relaxed);
                                            crate::SAVE_FLASH_REQUEST
                                                .store(true, core::sync::atomic::Ordering::Relaxed);

                                            send_server_ok!(uart, rx_buf, "9", tx_id.as_str());
                                            Timer::after_millis(200).await;
                                            break;
                                        }
                                        ServerCommand::DisableAuto(tx_id) => {
                                            info!(
                                                "GSM: Отключен АВТО режим (tx: {})",
                                                tx_id.as_str()
                                            );
                                            crate::AUTO_MODE.store(
                                                false,
                                                core::sync::atomic::Ordering::Relaxed,
                                            );
                                            crate::SAVE_FLASH_REQUEST
                                                .store(true, core::sync::atomic::Ordering::Relaxed);

                                            send_server_ok!(uart, rx_buf, "10", tx_id.as_str());
                                            Timer::after_millis(200).await;
                                            break;
                                        }
                                        ServerCommand::Unknown => {}
                                        _ => {
                                            info!(
                                                "Получена команда: {:?}",
                                                defmt::Debug2Format(&cmd)
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            error!("GSM: Ошибка подключения к TCP. Повтор через 10 сек...");
            crate::GSM_OK.store(false, core::sync::atomic::Ordering::Relaxed);
        }

        send_at!(uart, rx_buf, b"AT+CIPCLOSE=0\r\n", "OK", 2000);
        Timer::after_secs(10).await;
    }
}
