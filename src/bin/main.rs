#![no_std]
#![no_main]

use c6v3_lib::{
    ble::BlePacket,
    fuze::{detonate_fuze, RxTx},
    lc_cluster::{load_cell_reader, setup_load_cells},
    pressure::{pressure_reader, PressureReader},
};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    i2c::master::I2c,
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{AtCmdConfig, Config, RxConfig, Uart},
};
use esp_println::println;
use esp_wifi::{init, EspWifiController};
use static_cell::StaticCell;

// fifo_full_threshold (RX)
const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
const AT_CMD: u8 = 0x04;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let fuze = Output::new(
        peripherals.GPIO12,
        Level::Low,
        OutputConfig::default().with_pull(esp_hal::gpio::Pull::Down),
    );

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    println!("Wifi setup");

    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    let mut i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO23)
        .with_scl(peripherals.GPIO15)
        .into_async();
    println!("I2C setup");

    static BLE_SIGNAL: StaticCell<Signal<NoopRawMutex, BlePacket>> = StaticCell::new();
    let ble_signal = &*BLE_SIGNAL.init(Signal::new());

    spawner
        .spawn(c6v3_lib::ble::serial(
            esp_wifi_ctrl,
            peripherals.BT,
            ble_signal,
        ))
        .ok();
    println!("Bluetooth spawned");

    let (tx_pin, rx_pin) = (peripherals.GPIO16, peripherals.GPIO17);

    let config = Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(READ_BUF_SIZE as u16));

    let mut uart0 = Uart::new(peripherals.UART0, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();
    uart0.set_at_cmd(AtCmdConfig::default().with_cmd_char(AT_CMD));

    let (mut rx, tx) = uart0.split();

    match setup_load_cells(&mut i2c, &mut rx, ble_signal).await {
        Ok((lc_cluster, lc_params)) => {
            spawner
                .spawn(load_cell_reader(lc_cluster, lc_params, i2c))
                .ok();
        }
        Err(e) => {
            println!("Failed to setup load cell reader task: {e}");
        }
    }

    let pr = PressureReader::new(peripherals.GPIO2, peripherals.ADC1);
    spawner.spawn(pressure_reader(pr)).ok();

    spawner
        .spawn(detonate_fuze(fuze, RxTx { rx, tx }, ble_signal))
        .ok();

    loop {
        // println!("looping...");
        Timer::after_millis(1000).await;
    }
}
