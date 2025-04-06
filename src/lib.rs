#![no_std]

pub mod ble;
pub mod fuze;
pub mod lc_cluster;
pub mod load_cell;
pub mod math;
pub mod mux;
pub mod pressure;
pub mod util;

pub type AsyncI2C<'d> = esp_hal::i2c::master::I2c<'d, esp_hal::Async>;
