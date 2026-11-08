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
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer, with_timeout}; // Добавляем импорт модуля прерываний
// gsm
//use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed}; // Убедитесь, что импорты на месте

//
use core::sync::atomic::{AtomicBool, Ordering};
//
mod display;
//
use core::fmt::Write;
use display::DisplayCmd;

mod adc;
//
use embassy_stm32::adc::{Adc, AdcChannel}; // Обязательно для работы метода .degrade_adc()

mod encoder;
//
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
//PWM
use embassy_stm32::gpio::OutputType;
//use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};

// false = Внешнее питание (12В), true = Батарея
static POWER_SOURCE: AtomicBool = AtomicBool::new(false);
// Флаг ошибки заряда
static BATT_ERROR: AtomicBool = AtomicBool::new(false);
// Флаг стабильности питания (false = идут переходные процессы, измерения на паузе)
pub static POWER_STABLE: AtomicBool = AtomicBool::new(true);

use core::sync::atomic::AtomicI32;
//use core::sync::atomic::Ordering;
// Глобальная переменная для напряжения U1 (в милливольтах)
pub static U1_ACTUAL_MV: AtomicI32 = AtomicI32::new(0);
pub static I1_ACTUAL_MV: AtomicI32 = AtomicI32::new(0);
pub static P1_ACTUAL_MV: AtomicI32 = AtomicI32::new(0);

// Телеметрия для экранов
pub static DOOR_CLOSED: AtomicBool = AtomicBool::new(true);
pub static BAT_VOLTAGE_MV: AtomicU32 = AtomicU32::new(0);
pub static TEMP_C_X10: AtomicI32 = AtomicI32::new(0); // Градусы * 10
pub static GSM_SIGNAL: AtomicU32 = AtomicU32::new(0); // Сигнал CSQ
pub static GSM_OK: AtomicBool = AtomicBool::new(false); // Статус подключения

// PWM atomic
use core::sync::atomic::AtomicU32;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
//
// Глобальная переменная для чтения текущего логического ШИМ (0..1001)
pub static PWM1_ACTUAL: AtomicU32 = AtomicU32::new(0);
//
// Канал команд. Сюда DWIN и GSM будут кидать новые целевые значения (0..1001).
pub static PWM_CHANNEL: Channel<ThreadModeRawMutex, u32, 4> = Channel::new();

// --- Новые глобальные переменные ----------------------------------------------------------
// Энергия (кВт*ч). Храним f32 в виде u32 битов.
pub static ENERGY_KWH: AtomicU32 = AtomicU32::new(0);
// Наработка (часы)
pub static HOURS_WORK: AtomicU32 = AtomicU32::new(0);
// Второй ШИМ (ток)
pub static PWM2_ACTUAL: AtomicU32 = AtomicU32::new(333); // 1/3 от 1000
// Переключатель GSM
pub static GSM_ON: AtomicBool = AtomicBool::new(true);

// -------------------------------------
// Флаг запроса на сохранение во флеш (чтобы не писать из каждого места напрямую)
pub static SAVE_FLASH_REQUEST: AtomicBool = AtomicBool::new(false);

// Таймер отложенного сохранения (в секундах), чтобы не "затирать" флеш при вращении энкодера
pub static FLASH_SAVE_COUNTDOWN: AtomicU32 = AtomicU32::new(0);
//---------------------------------------

// --- Структура для сохранения (с Магическим словом) ---
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SavedConfig {
    pub magic: u32,
    pub energy_kwh: f32,
    pub hours_work: u32,
    pub pwm1_logical: u32,
    pub pwm2_logical: u32,
    pub gsm_on: u32, // 1 или 0
}

const MAGIC_WORD: u32 = 0xAA55AA55;
//----------------------------------------------------------------------------------------------

//bluetooth
mod bluetooth;

//rs485
mod rs485;

//Dwin display
mod dwin;

//Gsm
mod gsm;

// Явная привязка аппаратных линий прерываний к драйверу EXTI.
// Это требование новых версий для безопасной обработки IRQ под капотом.
bind_interrupts!(struct Irqs {
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
    EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
});

#[embassy_executor::task]
async fn monitor_power_good(mut pg: ExtiInput<'static, Async>, mut otg: Output<'static>) {
    otg.set_low();
    Timer::after_millis(100).await;

    let mut is_battery_mode;

    if pg.is_low() {
        POWER_SOURCE.store(false, Ordering::Relaxed);
        info!("[ПИТАНИЕ] СТАРТ: Внешнее напряжение 12В подключено! (PG = 0)");
        is_battery_mode = false;
    } else {
        POWER_SOURCE.store(true, Ordering::Relaxed);
        otg.set_high();
        warn!("[ПИТАНИЕ] СТАРТ: Работаем от АКБ! (PG = 1)");
        is_battery_mode = true;
    }

    loop {
        if is_battery_mode {
            Timer::after_secs(3).await;

            // --- НАЧАЛО НЕСТАБИЛЬНОЙ ЗОНЫ ---
            POWER_STABLE.store(false, Ordering::Relaxed); // Ставим АЦП на паузу
            otg.set_low();

            Timer::after_millis(35).await; // Снизили с 50 до 35 мс (предел по даташиту)

            if pg.is_low() {
                POWER_SOURCE.store(false, Ordering::Relaxed);
                is_battery_mode = false;
                info!("[ПИТАНИЕ] Внешнее напряжение 12В восстановлено! (PG = 0)");

                Timer::after_millis(350).await; // Даем 12В зарядить конденсаторы
                POWER_STABLE.store(true, Ordering::Relaxed); // Снимаем АЦП с паузы
            } else {
                otg.set_high(); // 12В нет, возвращаем 5В из АКБ
                Timer::after_millis(350).await; // Даем DC-DC время поднять напряжение до 5В
                POWER_STABLE.store(true, Ordering::Relaxed); // Снимаем АЦП с паузы
            }
            // --- КОНЕЦ НЕСТАБИЛЬНОЙ ЗОНЫ ---
        } else {
            pg.wait_for_high().await;
            Timer::after_millis(10).await;

            if pg.is_high() {
                POWER_STABLE.store(false, Ordering::Relaxed); // Пауза

                otg.set_high();
                POWER_SOURCE.store(true, Ordering::Relaxed);
                is_battery_mode = true;
                warn!("[ПИТАНИЕ] Отключение 12В! Переход на резервную АКБ! (PG = 1)");

                Timer::after_millis(20).await; // Ждем стабилизации 5В
                POWER_STABLE.store(true, Ordering::Relaxed); // Снимаем с паузы
            }
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

//задача изменения кВтч и ч раз в час и запрос на запись во флэш
#[embassy_executor::task]
pub async fn stats_task() {
    loop {
        // Спим 1 час
        Timer::after(embassy_time::Duration::from_secs(3600)).await;

        let u_mv = crate::U1_ACTUAL_MV.load(Ordering::Relaxed);
        let i_mv = crate::I1_ACTUAL_MV.load(Ordering::Relaxed);

        let u_v = (u_mv as f32) / 1000.0;
        let i_v = (i_mv as f32) / 1000.0;

        // Энергия за час: (U * I) / 1000 в кВт*ч
        let added_kwh = (u_v * i_v) / 1000.0;

        // Читаем, прибавляем, сохраняем обратно
        let current_bits = ENERGY_KWH.load(Ordering::Relaxed);
        let current_energy = f32::from_bits(current_bits);
        let new_energy = current_energy + added_kwh;
        ENERGY_KWH.store(new_energy.to_bits(), Ordering::Relaxed);

        // Инкремент часов
        HOURS_WORK.fetch_add(1, Ordering::Relaxed);

        // Даем сигнал менеджеру памяти записать данные во флеш
        SAVE_FLASH_REQUEST.store(true, Ordering::Relaxed);
    }
}

// Задача записи во флэш память (Аппаратный уровень STM32F4)
#[embassy_executor::task]
pub async fn flash_manager_task() {
    let flash_addr: u32 = 0x080E_0000;

    loop {
        Timer::after(embassy_time::Duration::from_secs(1)).await;

        // Обработка таймера отложенного сохранения
        let countdown = crate::FLASH_SAVE_COUNTDOWN.load(Ordering::Relaxed);
        if countdown > 0 {
            let next = countdown - 1;
            crate::FLASH_SAVE_COUNTDOWN.store(next, Ordering::Relaxed);

            // Если досчитали до нуля — даем команду на запись
            if next == 0 {
                SAVE_FLASH_REQUEST.store(true, Ordering::Relaxed);
            }
        }

        if SAVE_FLASH_REQUEST.load(Ordering::Relaxed) {
            SAVE_FLASH_REQUEST.store(false, Ordering::Relaxed);

            let config = SavedConfig {
                magic: MAGIC_WORD,
                energy_kwh: f32::from_bits(ENERGY_KWH.load(Ordering::Relaxed)),
                hours_work: HOURS_WORK.load(Ordering::Relaxed),
                pwm1_logical: crate::PWM1_ACTUAL.load(Ordering::Relaxed),
                pwm2_logical: crate::PWM2_ACTUAL.load(Ordering::Relaxed),
                gsm_on: if crate::GSM_ON.load(Ordering::Relaxed) {
                    1
                } else {
                    0
                },
            };

            // Готовим выровненный буфер (32 байта = 8 слов u32)
            let mut write_buf = [0xFFFF_FFFFu32; 8];
            unsafe {
                core::ptr::copy_nonoverlapping(
                    &config as *const _ as *const u8,
                    write_buf.as_mut_ptr() as *mut u8,
                    core::mem::size_of::<SavedConfig>(),
                );
            }

            info!("Сохранение во флеш (Прямой доступ к регистрам)...");

            unsafe {
                let flash_keyr = 0x40023C04 as *mut u32;
                let flash_sr = 0x40023C0C as *mut u32;
                let flash_cr = 0x40023C10 as *mut u32;

                // 1. Разблокировка флеш-памяти (Unlock)
                core::ptr::write_volatile(flash_keyr, 0x45670123);
                core::ptr::write_volatile(flash_keyr, 0xCDEF89AB);

                // Ждем готовности (BSY == 0)
                while (core::ptr::read_volatile(flash_sr) & (1 << 16)) != 0 {}

                // 2. Стирание сектора 11
                // SNB = 11 (биты 6:3) -> 11 << 3. SER = 1 (бит 1).
                let mut cr = core::ptr::read_volatile(flash_cr);
                cr &= !(0xF << 3); // Очищаем старый номер сектора
                cr |= (11 << 3) | (1 << 1);
                core::ptr::write_volatile(flash_cr, cr);

                // Старт стирания (STRT = бит 16)
                core::ptr::write_volatile(flash_cr, cr | (1 << 16));

                // Ждем завершения стирания
                while (core::ptr::read_volatile(flash_sr) & (1 << 16)) != 0 {}

                // Отключаем режим стирания
                cr = core::ptr::read_volatile(flash_cr);
                core::ptr::write_volatile(flash_cr, cr & !(1 << 1));

                // 3. Запись данных
                // PSIZE = 32-bit (биты 9:8) -> 2 << 8. PG = 1 (бит 0).
                cr = core::ptr::read_volatile(flash_cr);
                cr &= !(3 << 8);
                cr |= (2 << 8) | (1 << 0);
                core::ptr::write_volatile(flash_cr, cr);

                let flash_ptr = flash_addr as *mut u32;
                for i in 0..8 {
                    core::ptr::write_volatile(flash_ptr.add(i), write_buf[i]);
                    // Ждем завершения записи слова
                    while (core::ptr::read_volatile(flash_sr) & (1 << 16)) != 0 {}
                }

                // Отключаем запись
                cr = core::ptr::read_volatile(flash_cr);
                core::ptr::write_volatile(flash_cr, cr & !(1 << 0));

                // 4. Блокируем флеш (LOCK = бит 31)
                cr = core::ptr::read_volatile(flash_cr);
                core::ptr::write_volatile(flash_cr, cr | (1 << 31));
            }

            info!("Сохранение завершено!");
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

    // чтение из флэш памяти сохраненых настроек
    // Создаем драйвер флеш-памяти

    // чтение из флэш памяти сохраненых настроек
    let flash_addr: u32 = 0x080E_0000; // 1Mb (Сектор 11)

    // Читаем структуру напрямую из памяти
    let saved_config = unsafe { &*(flash_addr as *const SavedConfig) };

    if saved_config.magic == MAGIC_WORD {
        info!("Найдена валидная конфигурация во флеш-памяти!");
        // Восстанавливаем значения
        ENERGY_KWH.store(saved_config.energy_kwh.to_bits(), Ordering::Relaxed);
        HOURS_WORK.store(saved_config.hours_work, Ordering::Relaxed);
        crate::PWM1_ACTUAL.store(saved_config.pwm1_logical, Ordering::Relaxed);
        crate::PWM2_ACTUAL.store(saved_config.pwm2_logical, Ordering::Relaxed);
        crate::GSM_ON.store(saved_config.gsm_on == 1, Ordering::Relaxed);
    } else {
        info!("Флеш-память пуста или повреждена. Применяем настройки по умолчанию.");
        // Значения по умолчанию уже заданы при объявлении (PWM1=0, PWM2=333, GSM=true, etc.)

        // Сразу просим фоновую задачу сохранить дефолт во флеш
        SAVE_FLASH_REQUEST.store(true, Ordering::Relaxed);
    }

    // Задачи работы с флэш (без передачи аргумента flash)
    spawner.spawn(stats_task().unwrap());
    spawner.spawn(flash_manager_task().unwrap());

    //-----------------------------------------------------------

    // 1. Настройка светодиода (PB2)
    let led_pin = Output::new(p.PB2, Level::Low, Speed::Low);

    // 2. OTG (PC2) - Управление повышающим преобразователем.
    // Внешняя подтяжка к GND 10 кОм уже есть на плате.
    // 2. OTG (PC2) - Инициализируем в LOW, чтобы не блокировать 12В
    let otg_pin = Output::new(p.PC2, Level::Low, Speed::Low);

    // 3. PG (PC0) и STAT (PC1).
    // Внешние подтяжки 100 кОм к 3.3В, поэтому внутри МК используем Pull::None.
    // Передаем токен Irqs 4-м аргументом.
    let pg_pin = ExtiInput::new(p.PC0, p.EXTI0, Pull::None, Irqs);
    let stat_pin = ExtiInput::new(p.PC1, p.EXTI1, Pull::None, Irqs);

    // Инициализация пинов светодиодов
    let led_ext_pin = Output::new(p.PA15, Level::Low, Speed::Low);
    let led_batt_pin = Output::new(p.PD2, Level::Low, Speed::Low);

    // Init pin rele
    let _rele = Output::new(p.PC13, Level::Low, Speed::Low);

    // Init pins gsm
    // Инициализация обвязки GSM-модема A7682E
    let gsm_pwrkey = Output::new(p.PA8, Level::Low, Speed::Low);
    let gsm_reset = Output::new(p.PA11, Level::Low, Speed::Low);
    let gsm_dtr = Output::new(p.PC8, Level::Low, Speed::Low);
    //
    // Входы. Транслятор TXS0108E подтягивает уровни сам, поэтому Pull::None
    let gsm_status = Input::new(p.PA12, Pull::None);
    let gsm_ring = Input::new(p.PC9, Pull::None);

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

    //----------------------PWM-----------------------------
    // Инициализация ШИМ на TIM4 (10 кГц)
    // 1. Оборачиваем пины (без суффиксов каналов, просто .new)
    let pin_u1 = PwmPin::new(p.PB6, OutputType::PushPull);
    let pin_i1 = PwmPin::new(p.PB7, OutputType::PushPull);
    let pin_u2 = PwmPin::new(p.PB8, OutputType::PushPull);
    let pin_i2 = PwmPin::new(p.PB9, OutputType::PushPull);

    // 2. Создаем ШИМ
    let mut pwm = SimplePwm::new(
        p.TIM4,
        Some(pin_u1),
        Some(pin_i1),
        Some(pin_u2),
        Some(pin_i2),
        Hertz::khz(10),
        Default::default(),
    );

    // Обращаемся к каналам на лету (borrow сразу освобождается)
    pwm.ch1().enable();
    pwm.ch2().enable();
    pwm.ch3().enable();
    pwm.ch4().enable();

    pwm.ch1().set_duty_cycle(0);
    pwm.ch2().set_duty_cycle(0);
    pwm.ch3().set_duty_cycle(0);
    pwm.ch4().set_duty_cycle(0);

    // Получаем аппаратный потолок
    let max_duty = pwm.ch1().max_duty_cycle();
    let max_duty2 = pwm.ch2().max_duty_cycle();

    // 1. Восстанавливаем ШИМ1 (Напряжение)
    let pwm1_logical = crate::PWM1_ACTUAL.load(Ordering::Relaxed);
    let pwm1_hw = ((pwm1_logical as u64 * max_duty as u64) / 1001) as u32;
    pwm.ch1().set_duty_cycle(pwm1_hw);

    // 2. Восстанавливаем ШИМ2 (Ток). Считаем, что его шкала 0..1000
    let pwm2_logical = crate::PWM2_ACTUAL.load(Ordering::Relaxed);
    let pwm2_hw = ((pwm2_logical as u64 * max_duty2 as u64) / 1000) as u32;
    pwm.ch2().set_duty_cycle(pwm2_hw);

    // Остальные каналы пока в ноль
    pwm.ch3().set_duty_cycle(0);
    pwm.ch4().set_duty_cycle(0);

    //----------------------------------------------------------------------------------

    // Энкодер (аппаратный таймер TIM3)
    // Передаем QeiConfig::default() четвертым параметром!
    let qei = Qei::new(p.TIM3, p.PB4, p.PB5, QeiConfig::default());
    //
    let enc_btn = Input::new(p.PB3, Pull::None);
    //
    spawner.spawn(encoder::encoder_task(qei, enc_btn, pwm, max_duty).unwrap());

    // =========================================================
    // Запуск независимых задач
    // =========================================================
    // Макрос #[task] возвращает Result<SpawnToken, SpawnError>,
    // поэтому распаковываем токен (unwrap) перед передачей в spawner.
    spawner.spawn(blink_led(led_pin).unwrap());
    // Передаем otg_pin в задачу мониторинга
    spawner.spawn(monitor_power_good(pg_pin, otg_pin).unwrap());
    //
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

    // Отправляем начальный лог
    let mut msg = heapless::String::<32>::new();
    core::write!(&mut msg, "Booting system...").unwrap();
    display::DISPLAY_CHANNEL.send(DisplayCmd::Log(msg)).await;

    Timer::after_secs(2).await;

    // === ИНИЦИАЛИЗАЦИЯ ДАТЧИКОВ ===
    // Создаем драйверы прямо в main. Здесь компилятор автоматически разрешит все типы Peri!
    let adc_inst = Adc::new(p.ADC1);
    let door = Input::new(p.PA0, Pull::None);

    // Запускаем задачу
    spawner.spawn(
        adc::sensors_task(
            adc_inst,
            p.PA4.degrade_adc(),
            p.PA5.degrade_adc(),
            p.PA6.degrade_adc(),
            p.PA7.degrade_adc(),
            p.PC4.degrade_adc(),
            p.PC5.degrade_adc(),
            p.PB0.degrade_adc(),
            p.PB1.degrade_adc(),
            door,
        )
        .unwrap(),
    );

    // Запускаем задачу Bluetooth
    // Запускаем задачу Bluetooth (сначала USART, потом RX, потом TX)
    spawner.spawn(bluetooth::bluetooth_task(p.USART2, p.PA3, p.PA2).unwrap());

    // Запускаем RS485: USART3, RX (PC11), TX (PC10), RW (PC12)
    spawner.spawn(rs485::rs485_task(p.USART3, p.PC11, p.PC10, p.PC12).unwrap());

    // Запускаем DWIN LCD: USART6, RX (PC7), TX (PC6) + DMA каналы
    spawner.spawn(
        dwin::dwin_task(
            p.USART6, p.PC7, p.PC6, p.DMA2_CH1, // Канал DMA для приема (RX)
            p.DMA2_CH6, // Канал DMA для передачи (TX)
        )
        .unwrap(),
    );

    //Запускаем задачу GSM
    spawner.spawn(
        gsm::gsm_task(
            p.USART1, p.PA10, p.PA9, p.DMA2_CH2, p.DMA2_CH7, gsm_pwrkey, gsm_reset, gsm_dtr,
            gsm_status, gsm_ring,
        )
        .unwrap(),
    );

    loop {
        Timer::after_millis(5000).await;
    }
}
