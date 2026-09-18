#![no_std]
#![no_main]

use embassy_stm32::Config;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllPDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;

use defmt::{error, info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer, with_timeout}; // Добавляем импорт модуля прерываний
//
use core::sync::atomic::{AtomicBool, Ordering};
//
mod display;
use core::fmt::Write;
use display::{DisplayCmd, Telemetry};

// false = Внешнее питание (12В), true = Батарея
static POWER_SOURCE: AtomicBool = AtomicBool::new(false);
// Флаг ошибки заряда
static BATT_ERROR: AtomicBool = AtomicBool::new(false);

// Явная привязка аппаратных линий прерываний к драйверу EXTI.
// Это требование новых версий для безопасной обработки IRQ под капотом.
bind_interrupts!(struct Irqs {
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
    EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
});

// --- Задача 1: Мониторинг наличия сетевого питания (PG) ---
#[embassy_executor::task]
async fn monitor_power_good(mut pg: ExtiInput<'static, Async>) {
    // 1. ИНИЦИАЛИЗАЦИЯ: Читаем фактическое состояние при старте
    if pg.is_low() {
        POWER_SOURCE.store(false, Ordering::Relaxed);
        info!("[ПИТАНИЕ] СТАРТ: Внешнее напряжение подключено! (PG = 0)");
    } else {
        POWER_SOURCE.store(true, Ordering::Relaxed);
        warn!("[ПИТАНИЕ] СТАРТ: Работаем от АКБ! (PG = 1)");
    }

    // 2. РАБОЧИЙ ЦИКЛ
    loop {
        // Ждем любого перепада (события)
        pg.wait_for_any_edge().await;

        // Сначала ждем 50 мс, чтобы сигнал "успокоился" (антидребезг)
        Timer::after_millis(50).await;

        // Только теперь читаем стабилизировавшийся уровень
        if pg.is_low() {
            POWER_SOURCE.store(false, Ordering::Relaxed);
            info!("[ПИТАНИЕ] Внешнее напряжение подключено! (PG = 0)");
        } else {
            POWER_SOURCE.store(true, Ordering::Relaxed);
            warn!("[ПИТАНИЕ] Переход на резервную АКБ! (PG = 1)");
        }
    }
}

// --- Задача 2: Мониторинг статуса зарядки (STAT) ---
#[embassy_executor::task]
async fn monitor_charge_status(mut stat: ExtiInput<'static, Async>) {
    // ИНИЦИАЛИЗАЦИЯ
    if stat.is_low() {
        info!("[ЗАРЯД] СТАРТ: Идет зарядка АКБ (STAT = 0)");
    } else {
        info!("[ЗАРЯД] СТАРТ: Зарядка завершена / Сон (STAT = 1)");
    }

    loop {
        // 1. Спим и ждем любого изменения состояния (перепада)
        stat.wait_for_any_edge().await;

        // 2. Как только перепад случился, входим в цикл проверки "А не мигаем ли мы?"
        loop {
            // Ждем следующего перепада максимум 600 мс
            let next_edge =
                with_timeout(Duration::from_millis(600), stat.wait_for_any_edge()).await;

            match next_edge {
                Ok(_) => {
                    // Перепад случился быстрее чем за 600 мс! Это аппаратная ошибка.
                    if !BATT_ERROR.load(Ordering::Relaxed) {
                        BATT_ERROR.store(true, Ordering::Relaxed);
                        error!("[ЗАРЯД] Аппаратная ошибка BQ25606 (Мигание STAT)!");
                    }
                }
                Err(_) => {
                    // ТАЙМАУТ: Сигнал стабилизировался (перестал мигать).
                    // Снимаем флаг ошибки
                    BATT_ERROR.store(false, Ordering::Relaxed);

                    if stat.is_low() {
                        info!("[ЗАРЯД] Идет зарядка АКБ (STAT = 0)");
                    } else {
                        info!("[ЗАРЯД] Зарядка завершена / Сон (STAT = 1)");
                    }

                    // Выходим из цикла мигания, возвращаемся к бесконечному сну (шаг 1)
                    break;
                }
            }
        }
    }
}

// --- Задача 3: Системная индикация (LED) ---
#[embassy_executor::task]
async fn blink_led(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}

// --- Задача 3: Менеджер индикации ---
#[embassy_executor::task]
async fn led_manager(mut led_ext: Output<'static>, mut led_batt: Output<'static>) {
    let mut blink_state = false;

    loop {
        let is_batt = POWER_SOURCE.load(Ordering::Relaxed);
        let is_err = BATT_ERROR.load(Ordering::Relaxed);

        if !is_batt {
            // Внешнее питание 12В ЕСТЬ
            led_ext.set_high(); // PA15 всегда горит

            if is_err {
                // Ошибка АКБ: мигаем PD2
                if blink_state {
                    led_batt.set_high();
                } else {
                    led_batt.set_low();
                }
                blink_state = !blink_state;
                Timer::after_millis(500).await;
            } else {
                // Ошибки нет: просто гасим PD2
                led_batt.set_low();
                Timer::after_millis(100).await;
            }
        } else {
            // Внешнего питания НЕТ, работаем от АКБ
            led_ext.set_low(); // PA15 выключен
            led_batt.set_high(); // PD2 горит непрерывно
            Timer::after_millis(100).await;
        }
    }
}

//##########################################################################################
// --- Главный процесс ---
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 1. Создаем пустую конфигурацию
    let mut config = Config::default();

    // 2. Включаем внешний кварц на 8 МГц (HSE)
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });

    // 3. Настраиваем умножитель частоты (PLL)
    // Формула: Выходная частота = (HSE / prediv) * mul / divp
    // Вход VCO должен быть в пределах 1-2 МГц, поэтому делим 8 МГц на 4.
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: 4.into(),          // 8 MHz / 4 = 2 MHz (вход PLL)
        mul: 96.into(),            // 2 MHz * 96 = 192 MHz (внутренняя частота PLL)
        divp: Some(PllPDiv::DIV4), // 192 MHz / 4 = 48 MHz (наша системная частота!)
        divq: None,
        divr: None,
    });

    // 4. Указываем, что главным источником тактирования (SYSCLK) будет PLL
    config.rcc.sys = Sysclk::PLL1_P;

    // 5. Настраиваем делители для внутренних шин (чтобы периферия работала корректно)
    config.rcc.ahb_pre = AHBPrescaler::DIV1; // Шина AHB = 48 MHz (Процессор, DMA, GPIO)
    config.rcc.apb1_pre = APBPrescaler::DIV2; // Шина APB1 = 24 MHz (UART, Таймеры)
    config.rcc.apb2_pre = APBPrescaler::DIV1; // Шина APB2 = 48 MHz (SPI1/4, АЦП)

    // Принудительно разрешаем работу SWD-отладчика в спящем режиме
    config.enable_debug_during_sleep = true;

    // Применяем настройки железа
    let p = embassy_stm32::init(config);

    //-----------------------------------------------------------

    // 1. Настройка светодиода (PB2)
    let led_pin = Output::new(p.PB2, Level::Low, Speed::Low);

    // 2. OTG (PC2) - Управление повышающим преобразователем.
    // Внешняя подтяжка к GND 10 кОм уже есть на плате.
    let _otg_pin = Output::new(p.PC2, Level::Low, Speed::Low);

    // 3. PG (PC0) и STAT (PC1).
    // Внешние подтяжки 100 кОм к 3.3В, поэтому внутри МК используем Pull::None.
    // Передаем токен Irqs 4-м аргументом.
    let pg_pin = ExtiInput::new(p.PC0, p.EXTI0, Pull::None, Irqs);
    let stat_pin = ExtiInput::new(p.PC1, p.EXTI1, Pull::None, Irqs);

    // Инициализация пинов светодиодов
    let led_ext_pin = Output::new(p.PA15, Level::Low, Speed::Low);
    let led_batt_pin = Output::new(p.PD2, Level::Low, Speed::Low);

    info!("Станция катодной защиты: Система питания инициализирована!");

    // =========================================================
    // 2. ИНИЦИАЛИЗАЦИЯ ДИСПЛЕЯ
    // =========================================================
    let dc = Output::new(p.PB14, Level::Low, Speed::VeryHigh);
    let rst = Output::new(p.PB10, Level::High, Speed::VeryHigh);
    let cs = Output::new(p.PB12, Level::High, Speed::VeryHigh); // Программный CS

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = embassy_stm32::time::Hertz(4_000_000);

    let spi2 = embassy_stm32::spi::Spi::new_blocking_txonly(
        p.SPI2, p.PB13, // SCK
        p.PB15, // MOSI
        spi_config,
    );

    // Запускаем задачу
    spawner.spawn(display::display_task(spi2, dc, cs, rst).unwrap());

    // =========================================================
    // Запуск независимых задач
    // =========================================================
    // Макрос #[task] возвращает Result<SpawnToken, SpawnError>,
    // поэтому распаковываем токен (unwrap) перед передачей в spawner.
    spawner.spawn(blink_led(led_pin).unwrap());
    spawner.spawn(monitor_power_good(pg_pin).unwrap());
    spawner.spawn(monitor_charge_status(stat_pin).unwrap());
    // Запускаем менеджер индикации
    spawner.spawn(led_manager(led_ext_pin, led_batt_pin).unwrap());

    // --- Инициализация Bluetooth HLK-B40-I (Active-High Reset) ---
    let mut reset_bth_pin = Output::new(p.PA1, Level::High, Speed::Low);
    Timer::after_millis(500).await;
    reset_bth_pin.set_low();

    // =========================================================
    // 3. ТЕСТОВАЯ ОТПРАВКА ДАННЫХ
    // =========================================================

    //Timer::after_secs(1).await;
    //Timer::after_secs(1).await;

    // Отправляем начальный лог
    let mut msg = heapless::String::<32>::new();
    core::write!(&mut msg, "Booting system...").unwrap();
    display::DISPLAY_CHANNEL.send(DisplayCmd::Log(msg)).await;

    Timer::after_secs(2).await;

    // Отправляем тестовый пакет телеметрии
    // Формируем диагностическое сообщение
    let mut diag = heapless::String::<32>::new();
    core::write!(&mut diag, "SYSTEM OK").unwrap();

    let test_data = Telemetry {
        mod1_v: 12.4,
        mod1_i: 1.2,
        mod2_v: 11.8, // Добавили данные для красоты
        mod2_i: 0.5,  // Добавили данные для красоты
        p: -0.850,    // Изменили имя переменной
        is_door_open: false,
        gsm_signal_percent: 65,
        diag_msg: diag, // Передали сообщение
    };

    display::DISPLAY_CHANNEL
        .send(DisplayCmd::Update(test_data.clone()))
        .await;

    // СТРЕСС-ТЕСТ SPI
    let mut counter = 0.0;
    let mut is_open: bool = false;
    loop {
        let mut stress_data = test_data.clone();
        stress_data.mod1_v = counter; // Меняем цифры на экране, чтобы заставить SPI работать
        stress_data.is_door_open = is_open;
        stress_data.gsm_signal_percent = counter as u8;

        display::DISPLAY_CHANNEL
            .send(DisplayCmd::Update(stress_data))
            .await;

        counter += 0.1;
        if counter > 100.0 {
            counter = 0.0;
        }
        is_open = counter < 50.0;

        // Обновляем дисплей 50 раз в секунду (как в играх)
        Timer::after_millis(20).await;
    }
}
