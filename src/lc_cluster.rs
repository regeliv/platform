use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{uart::UartRx, Async};
use esp_println::println;
use thiserror::Error;

use crate::{
    ble::BlePacket,
    load_cell::{self, get_reading},
    math::RunningAverage,
    mux::{select_device, ControlRegister},
    util::{parse_float, read_line},
    AsyncI2C,
};

pub type LoadCellCluster = heapless::Vec<ControlRegister, 4>;

#[derive(Debug, Clone, Copy)]
pub struct LoadCellParams {
    baseline: f32,
    ratio: f32,
}

impl LoadCellParams {
    pub fn new(baseline: f32, weight: f32, weight_reading: f32, baseline_after_flip: f32) -> Self {
        LoadCellParams {
            baseline: baseline_after_flip,
            ratio: weight / (weight_reading - baseline),
        }
    }

    pub fn get_weight(&self, reading: f32) -> f32 {
        (reading - self.baseline) * self.ratio
    }
}

pub async fn get_load_cell_cluster(
    i2c: &mut AsyncI2C<'_>,
) -> Result<LoadCellCluster, esp_hal::i2c::master::Error> {
    let mut v = LoadCellCluster::new();

    for register in [
        ControlRegister::DEVICE0,
        ControlRegister::DEVICE1,
        ControlRegister::DEVICE2,
        ControlRegister::DEVICE3,
    ] {
        select_device(i2c, register).await?;
        match load_cell::init(i2c).await {
            Ok(_) => v.push(register).unwrap(),
            Err(e) => println!("nau7802 @ {:?} setup failure: {}", register, e),
        }
    }

    Ok(v)
}

async fn get_load_cell_mean(
    i2c: &mut AsyncI2C<'_>,
    load_cells: &LoadCellCluster,
    duration: Duration,
) -> Result<RunningAverage, esp_hal::i2c::master::Error> {
    let mut avg = RunningAverage::new();

    let start = Instant::now();
    while start.elapsed() < duration {
        let mut local_sum = 0.0f32;

        for &load_cell in load_cells {
            select_device(i2c, load_cell).await?;
            local_sum += load_cell::get_reading(i2c).await? as f32;
        }

        avg.update(local_sum);
    }

    Ok(avg)
}

async fn get_weight_from_uart(rx: &mut UartRx<'static, Async>) -> f32 {
    let mut buf = [0u8; 20];
    loop {
        let line = read_line(&mut buf, rx).await;
        match line {
            Err(e) => {
                println!("UART RxError {e}");
                continue;
            }
            Ok(line) => {
                if let Some(weight) = parse_float(&line.trim_ascii_end()) {
                    return weight;
                }
                println!("Failed to parse float, try again [UART]");
            }
        }
    }
}

async fn get_weight_from_ble(ble_signal: &'static Signal<NoopRawMutex, BlePacket>) -> f32 {
    loop {
        ble_signal.reset();
        let packet = ble_signal.wait().await;
        let slice = packet.as_slice().trim_ascii_end();

        match parse_float(slice) {
            Some(weight) => return weight,
            None => println!("Failed to parse float, try again [BLE]"),
        }
    }
}

async fn get_weight_user(
    rx: &mut UartRx<'static, Async>,
    ble_signal: &'static Signal<NoopRawMutex, BlePacket>,
) -> f32 {
    use embassy_futures::select::Either;
    let weight =
        embassy_futures::select::select(get_weight_from_uart(rx), get_weight_from_ble(ble_signal))
            .await;

    match weight {
        Either::First(w) => w,
        Either::Second(w) => w,
    }
}

#[derive(Error, Debug)]
pub enum LcClusterError {
    #[error("No load cells available")]
    NoLoadCells,
    #[error("{0}")]
    I2cError(#[from] esp_hal::i2c::master::Error),
}

pub async fn setup_load_cells(
    i2c: &mut AsyncI2C<'_>,
    rx: &mut UartRx<'static, Async>,
    ble_signal: &'static Signal<NoopRawMutex, BlePacket>,
) -> Result<(LoadCellCluster, LoadCellParams), LcClusterError> {
    println!("Setting up load cells");
    let cluster_lc = get_load_cell_cluster(i2c).await?;

    if cluster_lc.is_empty() {
        return Err(LcClusterError::NoLoadCells);
    }

    println!("Getting baseline for {:?}...", cluster_lc);

    let baseline = get_load_cell_mean(i2c, &cluster_lc, Duration::from_secs(5))
        .await?
        .get();

    println!("Baseline calculated. Place an object to calibrate the weight and input its weight once you've done so.");

    let weight_ref = get_weight_user(rx, &ble_signal).await;
    println!("Got reference weight: {weight_ref}");

    let reading_ref_weight = get_load_cell_mean(i2c, &cluster_lc, Duration::from_secs(5))
        .await?
        .get();

    println!("Reference weight reading: {reading_ref_weight}");
    println!("Once you've flipped the platform send 'offset' over serial or bluetooth");

    get_offset(rx, ble_signal).await;

    let baseline_after_flip = get_load_cell_mean(i2c, &cluster_lc, Duration::from_secs(5))
        .await?
        .get();

    println!("baseline: {baseline}, baseline after flip: {baseline_after_flip}, {weight_ref}kg: {reading_ref_weight}kg");

    let params = LoadCellParams::new(
        baseline,
        weight_ref,
        reading_ref_weight,
        baseline_after_flip,
    );

    Ok((cluster_lc, params))
}

async fn get_readings(
    i2c: &mut AsyncI2C<'_>,
    lc_cluster: &LoadCellCluster,
) -> Result<f32, esp_hal::i2c::master::Error> {
    let mut sum_readings = 0.0f32;
    for load_cell in lc_cluster {
        select_device(i2c, *load_cell).await?;
        sum_readings += get_reading(i2c).await? as f32;
    }
    Ok(sum_readings)
}

#[embassy_executor::task]
pub async fn load_cell_reader(
    lc_cluster: LoadCellCluster,
    lc_params: LoadCellParams,
    mut i2c: AsyncI2C<'static>,
) {
    loop {
        match get_readings(&mut i2c, &lc_cluster).await {
            Ok(reading) => println!(
                "lc,{},{}",
                Instant::now().as_millis(),
                lc_params.get_weight(reading)
            ),
            Err(e) => println!("I2C error: {e}"),
        }
        Timer::after_millis(15).await;
    }
}

async fn get_offset_signal_uart(rx: &mut UartRx<'static, Async>) {
    let mut buf = [0u8; 20];
    loop {
        let line = read_line(&mut buf, rx).await;
        match line {
            Err(e) => {
                println!("UART RxError {e}");
                continue;
            }
            Ok(line) => {
                if line.trim_ascii_end() == b"offset" {
                    return;
                }
                println!("[UART] Wrong input to start measuring offset - write: 'offset'");
            }
        }
    }
}

async fn get_offset_signal_ble(ble_signal: &'static Signal<NoopRawMutex, BlePacket>) {
    loop {
        ble_signal.reset();
        let buf = ble_signal.wait().await;
        if buf.as_slice().trim_ascii_end() == b"offset" {
            return;
        }
        println!("[BLE] Wrong input to start measuring offset - write: 'offset'");
    }
}

async fn get_offset(
    rx: &mut UartRx<'static, Async>,
    ble_signal: &'static Signal<NoopRawMutex, BlePacket>,
) {
    embassy_futures::select::select(
        get_offset_signal_uart(rx),
        get_offset_signal_ble(ble_signal),
    )
    .await;
}
