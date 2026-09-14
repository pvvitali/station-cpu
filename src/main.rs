#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer, with_timeout}; // Добавляем импорт модуля прерываний

// Явная привязка аппаратных линий прерываний к драйверу EXTI.
// Это требование новых версий для безопасной обработки IRQ под капотом.
bind_interrupts!(struct Irqs {
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
    EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
});

// --- Задача 1: Мониторинг наличия сетевого питания (PG) ---
#[embassy_executor::task]
async fn monitor_power_good(mut pg: ExtiInput<'static, Async>) {
    loop {
        pg.wait_for_any_edge().await;

        if pg.is_low() {
            defmt::info!("[ПИТАНИЕ] Внешнее напряжение подключено! (PG = 0)");
        } else {
            defmt::warn!("[ПИТАНИЕ] Переход на резервную АКБ! (PG = 1)");
        }

        Timer::after_millis(50).await;
    }
}

// --- Задача 2: Мониторинг статуса зарядки (STAT) ---
#[embassy_executor::task]
async fn monitor_charge_status(mut stat: ExtiInput<'static, Async>) {
    loop {
        stat.wait_for_any_edge().await;

        // Ждем не более 600 мс следующего перепада
        let next_edge = with_timeout(Duration::from_millis(600), stat.wait_for_any_edge()).await;

        match next_edge {
            Ok(_) => {
                defmt::error!("[ЗАРЯД] Аппаратная ошибка BQ25606 (Перегрев/Таймер/OVP)!");
                Timer::after_secs(2).await;
            }
            Err(_) => {
                if stat.is_low() {
                    defmt::info!("[ЗАРЯД] Идет зарядка АКБ (STAT = 0)");
                } else {
                    defmt::info!("[ЗАРЯД] Зарядка завершена / Сон (STAT = 1)");
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

// --- Главный процесс ---
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // 1. Настройка светодиода (PB2)
    let led_pin = Output::new(p.PB2, Level::Low, Speed::Low);

    // 2. OTG (PC2) - Управление повышающим преобразователем.
    // Внешняя подтяжка к GND 10 кОм уже есть на плате.
    let _otg = Output::new(p.PC2, Level::Low, Speed::Low);

    // 3. PG (PC0) и STAT (PC1).
    // Внешние подтяжки 100 кОм к 3.3В, поэтому внутри МК используем Pull::None.
    // Передаем токен Irqs 4-м аргументом.
    let pg_pin = ExtiInput::new(p.PC0, p.EXTI0, Pull::None, Irqs);
    let stat_pin = ExtiInput::new(p.PC1, p.EXTI1, Pull::None, Irqs);

    defmt::info!("Станция катодной защиты: Система питания инициализирована!");

    // 4. Запуск независимых задач
    // Макрос #[task] возвращает Result<SpawnToken, SpawnError>,
    // поэтому распаковываем токен (unwrap) перед передачей в spawner.
    spawner.spawn(blink_led(led_pin).unwrap());
    spawner.spawn(monitor_power_good(pg_pin).unwrap());
    spawner.spawn(monitor_charge_status(stat_pin).unwrap());

    loop {
        Timer::after_secs(1).await;
    }
}
