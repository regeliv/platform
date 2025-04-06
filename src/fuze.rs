use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_hal::{
    gpio::Output,
    uart::{UartRx, UartTx},
    Async,
};
use esp_println::println;

use crate::{ble::BlePacket, util::read_line};

pub struct RxTx {
    pub rx: UartRx<'static, Async>,
    pub tx: UartTx<'static, Async>,
}

async fn get_fuze_signal_uart(rx: &mut UartRx<'static, Async>) {
    let mut buf = [0u8; 20];
    loop {
        let line = read_line(&mut buf, rx).await;
        match line {
            Err(e) => {
                println!("UART RxError {e}");
                continue;
            }
            Ok(line) => {
                if line.trim_ascii_end() == b"fuze" {
                    return;
                }
                println!("[UART] Wrong input to start fuze - write: 'fuze'");
            }
        }
    }
}

async fn get_fuze_signal_ble(ble_signal: &'static Signal<NoopRawMutex, BlePacket>) {
    loop {
        ble_signal.reset();
        let buf = ble_signal.wait().await;
        if buf.as_slice().trim_ascii_end() == b"fuze" {
            return;
        }
        println!("[BLE] Wrong input to start fuze - write: 'fuze'");
    }
}

#[embassy_executor::task]
pub async fn detonate_fuze(
    mut fuze: Output<'static>,
    mut rx_tx: RxTx,
    ble_signal: &'static Signal<NoopRawMutex, BlePacket>,
) {
    println!("Write 'fuze' to detonate the fuze");

    embassy_futures::select::select(
        get_fuze_signal_uart(&mut rx_tx.rx),
        get_fuze_signal_ble(ble_signal),
    )
    .await;

    println!("Detonating");
    fuze.set_high();
}
