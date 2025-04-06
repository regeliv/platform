use crate::AsyncI2C;
use bitflags::{bitflags, Flags};
use embassy_time::{Duration, Instant, Timer};
use thiserror::Error;

bitflags! {
    pub struct PuCtrlFlags: u8 {
        const REGISTER_RESET      = 1 << 0;
        const POWER_UP_DIGITAL    = 1 << 1;
        const POWER_UP_ANALOG     = 1 << 2;
        const POWER_UP_READY      = 1 << 3;
        const CYCLE_START         = 1 << 4;
        const CYCLE_READY         = 1 << 5;
        const SYSTEM_CLOCK_SOURCE = 1 << 6;
        const AVDD_SOURCE         = 1 << 7;
    }
}

const NAU7802_ADDRESS: u8 = 0x2a;

type I2CResult<T> = Result<T, esp_hal::i2c::master::Error>;

#[derive(Error, Debug, PartialEq, Eq)]
pub enum LoadCellError {
    #[error("{0}")]
    I2CErr(#[from] esp_hal::i2c::master::Error),
    #[error("{0}")]
    SetupErr(#[from] SetupError),
}

#[derive(Error, Debug, PartialEq, Eq)]
pub enum SetupError {
    #[error("NAU7802 Timed out while powering it on")]
    PowerUpFail,
    #[error("NAU7802 Calibration timed out")]
    CalibrationTimeout,
    #[error("NAU7802 Calibration failed")]
    CalibrationFail,
}

pub async fn init(i2c: &mut AsyncI2C<'_>) -> Result<(), LoadCellError> {
    power_up(i2c).await?;

    init_adc_register(i2c).await?;

    set_ldo(i2c, Vldo::V3_3).await?;
    set_gain(i2c, Gain::X128).await?;
    set_conversion_rate(i2c, ConversionRate::SPS80).await?;

    init_power_control_register(i2c).await?;

    flush(i2c).await?;

    calibrate(i2c, CalMode::Internal, Duration::from_millis(1000)).await?;

    Ok(())
}

async fn write_reg(
    i2c: &mut AsyncI2C<'_>,
    reg: Register,
    val: u8,
) -> Result<(), esp_hal::i2c::master::Error> {
    i2c.write_async(NAU7802_ADDRESS, &[reg as u8, val]).await
}

async fn read_reg(
    i2c: &mut AsyncI2C<'_>,
    reg: Register,
) -> Result<u8, esp_hal::i2c::master::Error> {
    let mut result = [0u8];

    i2c.write_read_async(NAU7802_ADDRESS, &[reg as u8], &mut result)
        .await?;

    Ok(result[0])
}

async fn read_reg3(i2c: &mut AsyncI2C<'_>, starting_register: Register) -> I2CResult<[u8; 3]> {
    let mut result = [0u8; 3];
    i2c.write_read_async(NAU7802_ADDRESS, &[starting_register as u8], &mut result)
        .await?;

    Ok(result)
}

pub async fn power_up(i2c: &mut AsyncI2C<'_>) -> Result<(), LoadCellError> {
    // Power on sequence https://www.nuvoton.com/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf
    // 9.1.1

    write_reg(i2c, Register::PuCtrl, PuCtrlFlags::REGISTER_RESET.bits()).await?;
    //
    // 9.1.2
    write_reg(
        i2c,
        Register::PuCtrl,
        (PuCtrlFlags::POWER_UP_DIGITAL | PuCtrlFlags::POWER_UP_ANALOG).bits(),
    )
    .await?;

    // 9.1.3
    let mut counter = 0;
    loop {
        if counter >= 10 {
            return Err(LoadCellError::SetupErr(SetupError::PowerUpFail));
        }

        let pu_ctrl = PuCtrlFlags::from_bits_retain(read_reg(i2c, Register::PuCtrl).await?);
        if pu_ctrl.contains(PuCtrlFlags::POWER_UP_READY) {
            // 9.1.5, skipping 9.1.4 because it is not a step
            write_reg(
                i2c,
                Register::PuCtrl,
                pu_ctrl.union(PuCtrlFlags::CYCLE_START).bits(),
            )
            .await?;
            break;
        }

        Timer::after_millis(1).await;
        counter += 1;
    }

    Ok(())
}

pub async fn init_adc_register(i2c: &mut AsyncI2C<'_>) -> Result<(), esp_hal::i2c::master::Error> {
    let mut pga = read_reg(i2c, Register::Pga).await?;

    // ensure reading/writing from register 0x15 gets us the adc register (see 11.14)
    if pga & (1 << 7) != 0 {
        pga |= 1 << 7;
        write_reg(i2c, Register::Pga, pga).await?;
    }

    // see 9.4
    write_reg(i2c, Register::OtpB1OrAdc, 0x30).await?;

    Ok(())
}

async fn set_conversion_rate(i2c: &mut AsyncI2C<'_>, rate: ConversionRate) -> I2CResult<()> {
    let mut ctrl2 = read_reg(i2c, Register::Ctrl2).await?;

    ctrl2 &= 0b1000_1111;
    ctrl2 |= (rate as u8) << 4;

    write_reg(i2c, Register::Ctrl2, ctrl2).await?;

    Ok(())
}

async fn set_gain(i2c: &mut AsyncI2C<'_>, gain: Gain) -> I2CResult<()> {
    let mut ctrl1 = read_reg(i2c, Register::Ctrl1).await?;

    ctrl1 &= 0b1111_1000;
    ctrl1 |= gain as u8;
    write_reg(i2c, Register::Ctrl1, ctrl1).await?;

    Ok(())
}

async fn set_ldo(i2c: &mut AsyncI2C<'_>, ldo: Vldo) -> I2CResult<()> {
    let mut ctrl1 = read_reg(i2c, Register::Ctrl1).await?;

    // clear ldo
    ctrl1 &= 0b1100_0111;
    ctrl1 |= (ldo as u8) << 3;
    write_reg(i2c, Register::Ctrl1, ctrl1).await?;

    let mut pu_ctrl = PuCtrlFlags::from_bits_retain(read_reg(i2c, Register::PuCtrl).await?);
    pu_ctrl |= PuCtrlFlags::AVDD_SOURCE;

    write_reg(i2c, Register::PuCtrl, pu_ctrl.bits()).await?;

    Ok(())
}

async fn init_power_control_register(i2c: &mut AsyncI2C<'_>) -> I2CResult<()> {
    let mut power_control = read_reg(i2c, Register::PowerControl).await?;
    // follow SparkFun in setting this;
    power_control |= 1 << 7;

    write_reg(i2c, Register::PowerControl, power_control).await?;

    // unset ldo mode to 1, for improved accuracy;
    let mut pga = read_reg(i2c, Register::Pga).await?;
    pga &= !(1 << 6);
    write_reg(i2c, Register::Pga, pga).await?;

    // wait for LDO to stabilize, taken from SparkFun
    Timer::after_millis(250).await;

    Ok(())
}

pub async fn get_reading(i2c: &mut AsyncI2C<'_>) -> I2CResult<i32> {
    let [byte2, byte1, byte0] = read_reg3(i2c, Register::AdcoB2).await?;

    let mut result: u32 = ((byte2 as u32) << 16) | ((byte1 as u32) << 8) | (byte0 as u32);

    // convert from i24 signed to i32 signed, by sign extending
    if result >= 0x0080_0000 {
        result |= 0xFF00_0000;
    }

    let result = unsafe { core::mem::transmute::<u32, i32>(result) };

    Ok(result)
}

async fn flush(i2c: &mut AsyncI2C<'_>) -> I2CResult<()> {
    for _ in 0..100 {
        get_reading(i2c).await?;
    }

    Ok(())
}

async fn calibrate(
    i2c: &mut AsyncI2C<'_>,
    calibration_mode: CalMode,
    timeout: Duration,
) -> Result<(), LoadCellError> {
    let mut ctrl2 = read_reg(i2c, Register::Ctrl2).await?;

    // clear calmod
    ctrl2 &= 0b1111_1100;
    ctrl2 |= calibration_mode as u8;

    // trigger calibration
    ctrl2 |= 1 << 3;

    write_reg(i2c, Register::Ctrl2, ctrl2).await?;

    let mut get_calibration_status = async || -> I2CResult<CalStatus> {
        let ctrl2 = read_reg(i2c, Register::Ctrl2).await?;
        if ctrl2 & 0b0000_0100 == 0 {
            if ctrl2 & (1 << 3) != 0 {
                return Ok(CalStatus::Fail);
            } else {
                return Ok(CalStatus::Success);
            }
        }
        Ok(CalStatus::InProgress)
    };

    let start = Instant::now();
    let mut status = CalStatus::Fail;

    while start.elapsed() < timeout {
        status = get_calibration_status().await?;
        if status != CalStatus::InProgress {
            break;
        }
    }

    match status {
        CalStatus::InProgress => Err(SetupError::CalibrationTimeout.into()),
        CalStatus::Fail => Err(SetupError::CalibrationFail.into()),
        CalStatus::Success => Ok(()),
    }
}

#[allow(dead_code)]
enum Register {
    PuCtrl = 0x00,
    Ctrl1 = 0x01,
    Ctrl2 = 0x02,

    Ocal1B2 = 0x03,
    Ocal1B1 = 0x04,
    Ocal1B0 = 0x05,

    Gcal1B3 = 0x06,
    Gcal1B2 = 0x07,
    Gcal1B1 = 0x08,
    Gcal1B0 = 0x09,

    Ocal2B2 = 0x0A,
    Ocal2B1 = 0x0B,
    Ocal2B0 = 0x0C,

    Gcal2B3 = 0x0D,
    Gcal2B2 = 0x0E,
    Gcal2B1 = 0x0F,
    Gcal2B0 = 0x10,

    I2cControl = 0x11,

    AdcoB2 = 0x12,
    AdcoB1 = 0x13,
    AdcoB0 = 0x14,

    OtpB1OrAdc = 0x15,
    OtpB0 = 0x16,

    Pga = 0x1B,
    PowerControl = 0x1C,
    DeviceRevision = 0x1F,
}

/// SPS = Samples per second
#[allow(dead_code)]
enum ConversionRate {
    SPS10 = 0b000,
    SPS20 = 0b001,
    SPS40 = 0b010,
    SPS80 = 0b011,
    SPS320 = 0b111,
}

#[allow(dead_code)]
enum Gain {
    X1 = 0b000,
    X2 = 0b001,
    X4 = 0b010,
    X8 = 0b011,
    X16 = 0b100,
    X32 = 0b101,
    X64 = 0b110,
    X128 = 0b111,
}

#[allow(dead_code)]
enum Vldo {
    V4_5 = 0b000,
    V4_2 = 0b001,
    V3_9 = 0b010,
    V3_6 = 0b011,
    V3_3 = 0b100,
    V3_0 = 0b101,
    V2_7 = 0b110,
    V2_4 = 0b111,
}
#[allow(dead_code)]
#[derive(PartialEq, Eq, Debug)]
enum CalStatus {
    InProgress,
    Success,
    Fail,
}

#[allow(dead_code)]
enum CalMode {
    GainCalibration = 0b11,
    OffsetCalibration = 0b10,
    Internal = 0b00,
}
