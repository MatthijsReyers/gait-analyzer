
#[inline]
pub fn reg_to_f32(high: u8, low: u8) -> f32 {
    (i16::from_be_bytes([high, low])) as f32
}
