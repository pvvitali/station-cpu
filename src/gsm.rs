use core::str;
use defmt::{error, info};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{Peri, bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write;
use heapless::String;

use core::cell::RefCell;
use cortex_m::peripheral::SCB;
use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig};
use embassy_stm32::flash::{Flash, WRITE_SIZE};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

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

// Вспомогательная функция для точного округления при делении целых чисел
fn round_div(val: i32, div: i32) -> i32 {
    if val >= 0 {
        (val + div / 2) / div
    } else {
        (val - div / 2) / div
    }
}

macro_rules! send_at {
    ($uart:expr, $rx_buf:expr, $cmd:expr, $expected:expr, $wait_ms:expr) => {{
        // Вычитываем буфер досуха (пока не наступит таймаут)
        while let Ok(_) = with_timeout(
            Duration::from_millis(50),
            $uart.read_until_idle(&mut $rx_buf[..]),
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
                    $uart.read_until_idle(&mut $rx_buf[..]),
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
                    // БЕРЕМ МОДУЛЬ! Если прилетело -125, станет 1250 мВ. Если 125, тоже 1250.
                    target_u_mv: (target * 10).abs(),
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

const OTA_URL: &str = "http://sczprog.cloud/update.bin";
const MAX_FW_SIZE: usize = 256 * 1024; // Максимум = размер раздела ACTIVE (256 КБ)
const CHUNK_SIZE: usize = 512;

async fn download_and_flash_ota<DFU, STATE>(
    uart: &mut Uart<'static, embassy_stm32::mode::Async>,
    rx_buf: &mut [u8; 1024],
    updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
) -> Result<(), &'static str>
where
    DFU: embedded_storage::nor_flash::NorFlash,
    STATE: embedded_storage::nor_flash::NorFlash,
{
    info!("OTA: Старт обновления с {}", OTA_URL);

    // 1. Сброс старой HTTP-сессии и инициализация
    let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);
    if !send_at!(uart, rx_buf, b"AT+HTTPINIT\r\n", "OK", 3000) {
        return Err("HTTPINIT failed");
    }

    let mut url_cmd = heapless::String::<128>::new();
    core::fmt::write(
        &mut url_cmd,
        format_args!("AT+HTTPPARA=\"URL\",\"{}\"\r\n", OTA_URL),
    )
    .unwrap();
    if !send_at!(uart, rx_buf, url_cmd.as_bytes(), "OK", 3000) {
        let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);
        return Err("HTTPPARA URL failed");
    }

    // 2. GET-запрос: модем скачивает файл в свою оперативную память
    info!("OTA: Запрос файла (AT+HTTPACTION=0)...");
    while let Ok(_) = with_timeout(Duration::from_millis(50), uart.read_until_idle(rx_buf)).await {}
    if uart.write_all(b"AT+HTTPACTION=0\r\n").await.is_err() {
        return Err("UART write HTTPACTION failed");
    }

    let mut total_size: usize = 0;
    let start = embassy_time::Instant::now();
    while start.elapsed().as_secs() < 120 {
        if let Ok(Ok(len)) =
            with_timeout(Duration::from_secs(5), uart.read_until_idle(rx_buf)).await
        {
            if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                // Ожидаем: +HTTPACTION: 0,200,<размер>
                if let Some(idx) = resp.find("+HTTPACTION:") {
                    let tail = &resp[idx + 12..];
                    let parts: heapless::Vec<&str, 4> = tail.trim().split(',').collect();
                    if parts.len() >= 3 {
                        let status = parts[1].trim().parse::<u16>().unwrap_or(0);
                        let size_str = parts[2].split_whitespace().next().unwrap_or("0");
                        let size = size_str.parse::<usize>().unwrap_or(0);
                        drop(parts); // <-- Освобождаем rx_buf перед возможным вызовом send_at!
                        if status == 200 && size > 0 {
                            total_size = size;
                            break;
                        } else {
                            error!("OTA: HTTP ошибка, статус={}, размер={}", status, size);
                            let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);
                            return Err("HTTP status != 200");
                        }
                    }
                }
            }
        }
    }

    if total_size == 0 || total_size > MAX_FW_SIZE {
        error!("OTA: Некорректный размер файла: {} байт", total_size);
        let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);
        return Err("Invalid firmware size");
    }
    info!(
        "OTA: Файл скачан в модем ({} байт). Начинаем запись в DFU...",
        total_size
    );

    // 3. Постраничное чтение из модема и запись во флеш STM32
    let mut offset: usize = 0;
    let mut write_buf = [0xFFu8; CHUNK_SIZE];

    while offset < total_size {
        let to_read = core::cmp::min(CHUNK_SIZE, total_size - offset);

        // Очищаем мусор в UART
        while let Ok(_) =
            with_timeout(Duration::from_millis(20), uart.read_until_idle(rx_buf)).await
        {}

        let mut read_cmd = heapless::String::<48>::new();
        core::fmt::write(
            &mut read_cmd,
            format_args!("AT+HTTPREAD={},{}\r\n", offset, to_read),
        )
        .unwrap();
        if uart.write_all(read_cmd.as_bytes()).await.is_err() {
            return Err("UART write HTTPREAD failed");
        }

        // Собираем ответ модема в rx_buf, пока не получим заголовок + все `to_read` байт
        let mut total_rx = 0usize;
        let mut data_start: Option<usize> = None;
        let chunk_start = embassy_time::Instant::now();

        while chunk_start.elapsed().as_secs() < 10 {
            if total_rx >= rx_buf.len() {
                break;
            }
            if let Ok(Ok(n)) = with_timeout(
                Duration::from_millis(500),
                uart.read_until_idle(&mut rx_buf[total_rx..]),
            )
            .await
            {
                total_rx += n;

                // Ищем заголовок "+HTTPREAD:" и перенос строки '\n' сразу после него
                if data_start.is_none() {
                    if let Some(hdr_pos) = rx_buf[..total_rx]
                        .windows(10)
                        .position(|w| w == b"+HTTPREAD:")
                    {
                        if let Some(nl_rel) =
                            rx_buf[hdr_pos..total_rx].iter().position(|&b| b == b'\n')
                        {
                            data_start = Some(hdr_pos + nl_rel + 1);
                        }
                    }
                }

                if let Some(ds) = data_start {
                    if total_rx >= ds + to_read {
                        break;
                    }
                }
            }
        }

        let ds = match data_start {
            Some(pos) if total_rx >= pos + to_read => pos,
            _ => {
                error!("OTA: Таймаут чтения блока на смещении {}", offset);
                let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);
                return Err("HTTPREAD timeout");
            }
        };

        // Копируем данные и добиваем последний блок до кратности 4 байтам (WRITE_SIZE)
        write_buf.fill(0xFF);
        write_buf[..to_read].copy_from_slice(&rx_buf[ds..ds + to_read]);
        let aligned_len = (to_read + WRITE_SIZE - 1) & !(WRITE_SIZE - 1);

        if updater
            .write_firmware(offset, &write_buf[..aligned_len])
            .is_err()
        {
            error!("OTA: Ошибка записи во флеш на смещении {}", offset);
            let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);
            return Err("Flash write failed");
        }

        offset += to_read;
        if offset % (16 * 1024) == 0 || offset == total_size {
            info!("OTA: Записано {} / {} байт", offset, total_size);
        }
    }

    let _ = send_at!(uart, rx_buf, b"AT+HTTPTERM\r\n", "OK", 1000);

    // 4. Ставим флаг для загрузчика
    updater.mark_updated().map_err(|_| "mark_updated failed")?;
    info!("OTA: Прошивка записана и помечена! Перезагрузка...");
    Ok(())
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
    flash_peri: Peri<'static, peripherals::FLASH>,
) {
    let mut config = Config::default();
    config.baudrate = 115200;

    let mut uart = Uart::new(usart1, rx_pin, tx_pin, tx_dma, rx_dma, Irqs, config).unwrap();
    let mut rx_buf = [0u8; 1024];

    let layout = Flash::new_blocking(flash_peri).into_blocking_regions();
    let state_flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(layout.bank1_region2));
    let dfu_flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(layout.bank1_region3));

    let fw_config = FirmwareUpdaterConfig::from_linkerfile_blocking(&dfu_flash, &state_flash);
    let mut magic = AlignedBuffer([0; WRITE_SIZE]);
    let mut updater = BlockingFirmwareUpdater::new(fw_config, &mut magic.0);

    // Подтверждаем загрузчику, что текущая прошивка успешно стартовала
    match updater.mark_booted() {
        Ok(_) => info!("OTA: Текущая прошивка подтверждена (mark_booted OK)."),
        Err(_) => error!("OTA: Ошибка mark_booted!"),
    }

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

    // --- НОВАЯ ПЕРЕМЕННАЯ ДЛЯ УПРАВЛЕНИЯ SIM-КАРТАМИ ---
    let mut active_sim = 0; // 0 = SIM1, 1 = SIM2

    loop {
        // --- СБРОС ФЛАГОВ ПЕРЕД НОВОЙ ПОПЫТКОЙ РЕГИСТРАЦИИ ---
        crate::GSM_OK.store(false, core::sync::atomic::Ordering::Relaxed);

        // ========================================================
        // --- ОБРАБОТКА ВЫКЛЮЧЕННОГО СОСТОЯНИЯ ---
        if !crate::GSM_ON.load(core::sync::atomic::Ordering::Relaxed) {
            info!("GSM: Работа остановлена. Выключаем модем...");

            // Пробуем выключить программно
            let _ = uart.write_all(b"AT+CPOWD=1\r\n").await;
            Timer::after_secs(3).await;

            // Если пин STATUS все еще высокий - глушим аппаратно через PWRKEY
            if status.is_high() {
                pwrkey.set_high();
                Timer::after_millis(2500).await; // Для A7682E удержание > 2.5 сек выключает модуль
                pwrkey.set_low();
            }

            // --- БЕСКОНЕЧНЫЙ СОН, ПОКА НЕ ВКЛЮЧАТ ---
            while !crate::GSM_ON.load(core::sync::atomic::Ordering::Relaxed) {
                Timer::after_secs(1).await;
            }

            info!("GSM: Включение модема с экрана...");
            // Запускаем аппаратно
            if status.is_low() {
                pwrkey.set_high();
                Timer::after_millis(1000).await;
                pwrkey.set_low();
                Timer::after_secs(3).await;
            }
            // Пропускаем текущий виток цикла, чтобы переинициализировать AT-команды
            continue;
        }
        // ========================================================

        info!(
            "GSM: Регистрация в сети и настройка (Слот {})...",
            active_sim + 1
        );

        let mut is_registered = false;
        let mut reg_attempts = 0;
        let mut cpin_errors = 0;

        loop {
            let _ =
                with_timeout(Duration::from_millis(50), uart.read_until_idle(&mut rx_buf)).await;

            // 1. Проверяем физическое наличие SIM-карты
            let mut sim_ready = false;
            let _ = uart.write_all(b"AT+CPIN?\r\n").await;
            if let Ok(Ok(len)) = with_timeout(
                Duration::from_millis(1000),
                uart.read_until_idle(&mut rx_buf),
            )
            .await
            {
                if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                    if resp.contains("READY") {
                        sim_ready = true;
                        cpin_errors = 0; // Карта на месте, сбрасываем счетчик ошибок
                    } else if resp.contains("ERROR") {
                        cpin_errors += 1; // Карты нет или она не читается
                    }
                }
            }

            // БЫСТРЫЙ СБРОС: Если физически нет SIM-карты, переключаемся через 3sec(3*1) (~9 секунд3*3)
            if cpin_errors >= 1 {
                error!("GSM: SIM-карта не обнаружена! Ускоренная смена слота...");
                break;
            }

            // 2. Если карта на месте, проверяем регистрацию в сети
            if sim_ready {
                let _ = uart.write_all(b"AT+CEREG?\r\n").await;
                if let Ok(Ok(len)) = with_timeout(
                    Duration::from_millis(1000),
                    uart.read_until_idle(&mut rx_buf),
                )
                .await
                {
                    if let Ok(resp) = str::from_utf8(&rx_buf[..len]) {
                        if resp.contains("+CEREG: 0,1")
                            || resp.contains("+CEREG: 0,5")
                            || resp.contains("+CEREG: 1,1")
                            || resp.contains("+CEREG: 1,5")
                        {
                            is_registered = true;
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
                        if resp.contains("+CREG: 0,1") || resp.contains("+CREG: 0,5") {
                            is_registered = true;
                        }
                    }
                }
            }

            if is_registered {
                break;
            }

            // Счетчик неудачных попыток регистрации в сети
            reg_attempts += 1;
            if reg_attempts >= 13 {
                // ~40 секунд на поиск сети при вставленной SIM
                error!("GSM: Тайм-аут регистрации в сети! Смена SIM-карты...");
                break;
            }
            Timer::after_secs(3).await;
        }

        // --- ЛОГИКА ПЕРЕКЛЮЧЕНИЯ SIM ПРИ ОШИБКЕ ---
        if !is_registered {
            active_sim = if active_sim == 0 { 1 } else { 0 };

            let mut cmd = heapless::String::<32>::new();
            core::fmt::write(
                &mut cmd,
                format_args!("AT*SELECTSIMSLOT={},1\r\n", active_sim),
            )
            .unwrap();
            let _ = send_at!(uart, rx_buf, cmd.as_bytes(), "OK", 2000);

            info!(
                "GSM: Перезагрузка радиомодуля для слота {}...",
                active_sim + 1
            );
            let _ = send_at!(uart, rx_buf, b"AT+CFUN=0\r\n", "OK", 5000);
            Timer::after_secs(2).await;
            let _ = send_at!(uart, rx_buf, b"AT+CFUN=1\r\n", "OK", 5000);
            Timer::after_secs(4).await; // Даем модулю время загрузить SIM

            continue;
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
        info!("GSM: Ожидание стабилизации уровня сигнала (CSQ)...");
        let mut csq_attempts = 0;

        while csq_attempts < 10 {
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
                                // Если сигнал не 99 (известен) или мы уже слишком долго ждем
                                if csq_val != 99 || csq_attempts == 9 {
                                    let valid_csq = if csq_val == 99 { 0 } else { csq_val };
                                    crate::GSM_SIGNAL
                                        .store(valid_csq, core::sync::atomic::Ordering::Relaxed);
                                    info!("GSM: Сигнал получен: {}", valid_csq);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            csq_attempts += 1;
            Timer::after_millis(500).await; // Ждем полсекунды перед следующей попыткой
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

            'telemetry_loop: loop {
                // Читаем реальные данные с датчиков
                let u1_mv = crate::U1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let i1_mv = crate::I1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let p1_mv = crate::P1_ACTUAL_MV.load(core::sync::atomic::Ordering::Relaxed);
                let bat_mv = crate::BAT_VOLTAGE_MV.load(core::sync::atomic::Ordering::Relaxed);
                let temp_x10 = crate::TEMP_C_X10.load(core::sync::atomic::Ordering::Relaxed);
                let csq_raw = crate::GSM_SIGNAL.load(core::sync::atomic::Ordering::Relaxed);

                // 0 - HardwareId (теперь берем из памяти)
                let id_val = crate::STATION_ID.load(core::sync::atomic::Ordering::Relaxed);
                let mut id_str = heapless::String::<16>::new();
                core::fmt::write(&mut id_str, format_args!("{}", id_val)).unwrap();

                // 1 - Выходной ток * 10 (мА / 100) -> 30.2A = 302
                let i_str = round_div(i1_mv, 100);

                // 2 - Выходное напряжение * 10 (мВ / 100) -> -21.5V = -215
                let u_str = round_div(u1_mv, 100);

                // 3 - Защитный потенциал * 100 (мВ / 10) -> 8.3V = 830
                let p_str = round_div(p1_mv, 10);

                // 4 - Напряжение батареи * 10 (мВ / 100) -> 4.2V = 42
                let bat_str = round_div(bat_mv as i32, 100);

                // сырые 0..31
                let gsm_percent = csq_raw;

                // 6 - Наличие 220V (1 - есть, 0 - нет)
                let is_on_battery = crate::POWER_SOURCE.load(core::sync::atomic::Ordering::Relaxed);
                let is_220v_ok = if !is_on_battery { "1" } else { "0" };

                // 7 - Дверь (0 - закрыта, 1 - открыта)
                let is_door_closed = crate::DOOR_CLOSED.load(core::sync::atomic::Ordering::Relaxed);
                let door_str = if is_door_closed { "0" } else { "1" };

                // 8 - Температура (целые градусы)
                let temp_c = round_div(temp_x10, 10);

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

                // 14 - Целевой потенциал (только положительное число)
                let target_mv = crate::AUTO_TARGET_MV
                    .load(core::sync::atomic::Ordering::Relaxed)
                    .abs();
                let auto_value = round_div(target_mv, 10);

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
                    // --- ПРОВЕРКА ВЫКЛЮЧЕНИЯ В РЕАЛЬНОМ ВРЕМЕНИ ---
                    if !crate::GSM_ON.load(core::sync::atomic::Ordering::Relaxed) {
                        info!("GSM: Выключен с экрана! Разрыв связи...");
                        break 'telemetry_loop; // ИСПРАВЛЕНИЕ: Выходим из внешнего цикла телеметрии
                    }

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

                                            // ИЩЕМ ПОСЛЕДНЮЮ АКТУАЛЬНУЮ ЗАПИСЬ ВО ФЛЕШКЕ
                                            let mut restore_pwm = 0;
                                            let flash_base =
                                                0x080E_0000 as *const crate::SavedConfig;
                                            for i in 0..2720 {
                                                let config = unsafe { &*flash_base.add(i) };
                                                if config.magic == 0xAA55AA55 {
                                                    restore_pwm = config.pwm1_logical;
                                                } else if config.magic == 0xFFFFFFFF {
                                                    break; // Нашли конец записей
                                                }
                                            }

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
                                        ServerCommand::UpdateFlash(tx_id) => {
                                            info!(
                                                "GSM: Команда Обновления прошивки (tx: {})",
                                                tx_id.as_str()
                                            );
                                            send_server_ok!(uart, rx_buf, "15", tx_id.as_str());
                                            Timer::after_millis(500).await;

                                            // Закрываем TCP и ставим на паузу запись настроек во флеш
                                            let _ = send_at!(
                                                uart,
                                                rx_buf,
                                                b"AT+CIPCLOSE=0\r\n",
                                                "OK",
                                                2000
                                            );
                                            crate::OTA_IN_PROGRESS
                                                .store(true, core::sync::atomic::Ordering::Relaxed);

                                            match download_and_flash_ota(
                                                &mut uart,
                                                &mut rx_buf,
                                                &mut updater,
                                            )
                                            .await
                                            {
                                                Ok(_) => {
                                                    Timer::after_millis(500).await;
                                                    SCB::sys_reset(); // Уходим в перезагрузку -> bootloader обновит прошивку
                                                }
                                                Err(e) => {
                                                    error!("OTA: Ошибка обновления: {}", e);
                                                    crate::OTA_IN_PROGRESS.store(
                                                        false,
                                                        core::sync::atomic::Ordering::Relaxed,
                                                    );
                                                    crate::GSM_OK.store(
                                                        false,
                                                        core::sync::atomic::Ordering::Relaxed,
                                                    );
                                                    break 'telemetry_loop; // Переподключаемся к TCP-серверу
                                                }
                                            }
                                        }
                                        ServerCommand::Unknown => {} //
                                                                     // _ => {
                                                                     //     info!(
                                                                     //         "Получена команда: {:?}",
                                                                     //         defmt::Debug2Format(&cmd)
                                                                     //     );
                                                                     // }
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
