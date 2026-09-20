use core::fmt::Write;
use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::ADC1;
use embassy_time::Timer;

use micromath::F32Ext; // Обязательно для вычисления логарифма (ln)

const TEMP_T1_OFFSET: f32 = -2.0;

fn raw_to_volts(raw: u16) -> f32 {
    (raw as f32 / 4095.0) * 3.3
}

fn calc_real_voltage(adc_v: f32) -> f32 {
    // 1. Считаем базовое теоретическое значение
    let v_theoretical = adc_v * 30.125;

    // 2. Применяем линейную калибровку по измеренным точкам
    // Формула: Y = k * X + b
    let mut v_real = (v_theoretical * 1.02145) - 0.32686;

    // 3. Отсекаем микро-шумы ниже нуля (чтобы на экране не было -0.01V)
    if v_real < 0.0 {
        v_real = 0.0;
    }

    v_real
}

fn calc_real_current(adc_v: f32) -> f32 {
    let k_theoretical = 12.5;

    // Идеальный наклон, просто вычитаем константу смещения 0.11 А
    let mut i_real = (adc_v * k_theoretical) - 0.11;

    // Отсечка нуля (если ток меньше 0.10 А, считаем это тепловым шумом)
    if i_real < 0.05 {
        i_real = 0.0;
    }

    i_real
}

fn calc_temperature(raw_adc: u16) -> f32 {
    // Защита от деления на ноль (если пин замкнут на питание или землю)
    if raw_adc == 0 || raw_adc >= 4095 {
        return -99.9;
    }

    // 1. Вычисляем сопротивление термистора R_ntc
    // Верхний резистор R1 = 10000 Ом. Формула делителя выведена через raw_adc.
    let r1 = 10000.0;
    let r_ntc = r1 * (raw_adc as f32) / (4095.0 - raw_adc as f32);

    // 2. Константы термистора NTCS0805E3103JHT
    let r0 = 10000.0; // 10 кОм при 25°C
    let t0 = 298.15; // 25°C в Кельвинах
    let beta = 3960.0; // B-коэффициент из даташита

    // 3. Формула B-параметра
    let inv_t = (1.0 / t0) + (1.0 / beta) * (r_ntc / r0).ln();
    let temp_k = 1.0 / inv_t;

    // 4. Переводим Кельвины в Цельсии
    temp_k - 273.15
}

#[embassy_executor::task]
pub async fn sensors_task(
    mut adc: Adc<'static, ADC1>, // Принимаем готовый АЦП
    mut pin_p: AnyAdcChannel<'static, ADC1>,
    mut pin_m1_v: AnyAdcChannel<'static, ADC1>,
    mut pin_m1_i: AnyAdcChannel<'static, ADC1>,
    mut pin_m2_v: AnyAdcChannel<'static, ADC1>,
    mut pin_m2_i: AnyAdcChannel<'static, ADC1>,
    mut pin_t1: AnyAdcChannel<'static, ADC1>,
    mut pin_t2: AnyAdcChannel<'static, ADC1>,
    mut pin_bat: AnyAdcChannel<'static, ADC1>,
    pin_door: Input<'static>, // Принимаем готовый вход двери
) {
    let sample_time = SampleTime::CYCLES112;

    let mut i1_filtered: f32 = 0.0;
    let mut i2_filtered: f32 = 0.0;

    loop {
        // Читаем АЦП
        let raw_p = adc.blocking_read(&mut pin_p, sample_time);
        let raw_m1_v = adc.blocking_read(&mut pin_m1_v, sample_time);
        let raw_m1_i = adc.blocking_read(&mut pin_m1_i, sample_time);
        let raw_m2_v = adc.blocking_read(&mut pin_m2_v, sample_time);
        let raw_m2_i = adc.blocking_read(&mut pin_m2_i, sample_time);
        let raw_bat = adc.blocking_read(&mut pin_bat, sample_time);
        let raw_t1 = adc.blocking_read(&mut pin_t1, sample_time);
        let _raw_t2 = adc.blocking_read(&mut pin_t2, sample_time); // Просто чтение для примера

        let current_door_state = pin_door.is_low();

        let voltage_p = raw_to_volts(raw_p);

        // 1. Получаем напряжение на ножке АЦП (0..3.3 В)
        let adc_v1 = raw_to_volts(raw_m1_v);
        let adc_v2 = raw_to_volts(raw_m2_v);

        // 2. Переводим в реальные Вольты (0..50 В)
        let voltage_m1 = calc_real_voltage(adc_v1);
        let voltage_m2 = calc_real_voltage(adc_v2);

        // 3. Получаем напряжение на ножке АЦП (0..3.3 В)
        let adc_i1 = raw_to_volts(raw_m1_i);
        let adc_i2 = raw_to_volts(raw_m2_i);

        // 4. Переводим в реальные Амперы (0..30 А)
        let current_m1_raw = calc_real_current(adc_i1);
        let current_m2_raw = calc_real_current(adc_i2);

        // ФИЛЬТРАЦИЯ (например, 80% старого значения и 20% нового)
        // Чем меньше коэффициент у нового значения (0.2), тем сильнее сглаживание
        i1_filtered = (i1_filtered * 0.8) + (current_m1_raw * 0.2);
        i2_filtered = (i2_filtered * 0.8) + (current_m2_raw * 0.2);

        // Вычисляем реальное напряжение батареи через делитель 100к / 220к
        let bat_voltage = raw_to_volts(raw_bat) * (320.0 / 220.0);

        // Получаем температуру в градусах Цельсия
        let temp_c1 = calc_temperature(raw_t1) + TEMP_T1_OFFSET; // 2.0 - Калибровочный сдвиг под точку 25°C

        let mut diag_buf = heapless::String::new();
        // Записываем bat_voltage вместо raw_to_volts(raw_bat) * 3.0
        core::write!(&mut diag_buf, "BAT: {:.2}V T1:{:.0}C", bat_voltage, temp_c1).unwrap();

        let telemetry_data = crate::display::Telemetry {
            mod1_v: voltage_m1,
            mod1_i: i1_filtered,
            mod2_v: voltage_m2,
            mod2_i: i2_filtered,
            p: voltage_p,
            is_door_open: current_door_state,
            gsm_signal_percent: 85,
            diag_msg: diag_buf,
        };

        // let _ = crate::display::DISPLAY_CHANNEL
        //     .try_send(crate::display::DisplayCmd::Update(telemetry_data));
        crate::display::DISPLAY_CHANNEL
            .send(crate::display::DisplayCmd::Update(telemetry_data))
            .await;

        Timer::after_millis(200).await;
    }
}
