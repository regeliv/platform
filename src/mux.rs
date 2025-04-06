use bitflags::bitflags;

use crate::AsyncI2C;

const PCA9546A_ADDR: u8 = 0x70;

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct ControlRegister: u8 {
        const DEVICE0 = 1 << 0;
        const DEVICE1 = 1 << 1;
        const DEVICE2 = 1 << 2;
        const DEVICE3 = 1 << 3;
    }
}

pub async fn select_device(
    i2c: &mut AsyncI2C<'_>,
    control_register: ControlRegister,
) -> Result<(), esp_hal::i2c::master::Error> {
    i2c.write_async(PCA9546A_ADDR, &[control_register.bits()])
        .await?;

    Ok(())
}

pub async fn get_selected(
    i2c: &mut AsyncI2C<'_>,
) -> Result<ControlRegister, esp_hal::i2c::master::Error> {
    let mut buf = [0u8; 1];
    i2c.read_async(PCA9546A_ADDR, &mut buf).await?;

    Ok(ControlRegister::from_bits_truncate(buf[0]))
}
