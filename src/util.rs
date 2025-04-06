use esp_hal::{
    uart::{RxError, UartRx},
    Async,
};
use esp_println::print;

pub fn parse_float(buf: &[u8]) -> Option<f32> {
    use core::str::FromStr;
    let s = core::str::from_utf8(buf).ok()?;

    f32::from_str(s).ok()
}

pub async fn read_line<'a>(
    buf: &'a mut [u8],
    rx: &mut UartRx<'static, Async>,
) -> Result<&'a mut [u8], RxError> {
    let mut offset = 0;
    loop {
        let bytes_read = embedded_io_async::Read::read(rx, &mut buf[offset..]).await?;
        if bytes_read == 0 {
            return Ok(&mut buf[..offset]);
        }

        print!("{}", unsafe {
            core::str::from_utf8_unchecked(&buf[offset..offset + bytes_read])
        });

        if let Some(ret_position) = buf[offset..offset + bytes_read]
            .iter()
            .position(|&c| c == b'\r')
        {
            return Ok(&mut buf[..offset + ret_position]);
        }
        offset += bytes_read;
    }
}
