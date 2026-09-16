#![no_std]
#![no_main]

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

// --- Главный процесс ---
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 1. Создаем конфигурацию по умолчанию
    let mut config = embassy_stm32::Config::default();

    // 2. Принудительно разрешаем работу SWD-отладчика в спящем режиме
    config.enable_debug_during_sleep = true;

    // 3. Инициализируем контроллер с этой настройкой
    let p = embassy_stm32::init(config);

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

    // 4. Запуск независимых задач
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

    loop {
        Timer::after_secs(1).await;
    }
}
