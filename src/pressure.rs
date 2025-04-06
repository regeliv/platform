use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::GpioPin,
    peripherals::ADC1,
    Async,
};
use esp_println::println;

use crate::math::RunningMin;

pub struct PressureReader<'a> {
    adc_pin: AdcPin<GpioPin<2>, ADC1>,
    adc1: Adc<'a, ADC1, Async>,
}

impl<'a> PressureReader<'a> {
    pub fn new(pin: GpioPin<2>, adc: ADC1) -> Self {
        let mut adc1_config = AdcConfig::new();
        let adc_pin = adc1_config.enable_pin(pin, Attenuation::_11dB);
        let adc1 = Adc::new(adc, adc1_config).into_async();

        Self { adc_pin, adc1 }
    }

    pub async fn read(&mut self) -> u16 {
        self.adc1.read_oneshot(&mut self.adc_pin).await
    }
}

fn voltage_from_reading(reading: u16) -> f32 {
    return (reading as f32) * 0.003 - 6.53;
}

async fn get_offset(pr: &mut PressureReader<'_>) -> f32 {
    let mut running_min = RunningMin::new();

    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(5) {
        let reading = pr.read().await;
        running_min.update(voltage_from_reading(reading));
    }

    let offset = running_min.get().unwrap();
    println!("Pressure sensor offset: {}", offset);

    offset
}

fn get_pressure(voltage: f32, offset: f32) -> f32 {
    const MAX_PRESSURE_IN_KPA: f32 = 1_600f32;
    const KPA_PER_VOLT: f32 = MAX_PRESSURE_IN_KPA / 4.0;

    (voltage - offset) * KPA_PER_VOLT
}

#[embassy_executor::task]
pub async fn pressure_reader(mut pr: PressureReader<'static>) {
    let offset = get_offset(&mut pr).await;

    loop {
        let reading = pr.read().await;
        let pressure = get_pressure(voltage_from_reading(reading), offset);

        println!("pr,{},{}", Instant::now().as_millis(), pressure);
        Timer::after_millis(15).await;
    }
}
