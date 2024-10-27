
#[inline]
pub fn reg_to_f32(high: u8, low: u8) -> f32 {
    (i16::from_be_bytes([high, low])) as f32
}

#[inline]
pub fn raw_i16_to_u16(x: i16) -> u16 {
    unsafe {
        core::mem::transmute(x)
    }
}


#[inline]
pub fn raw_u16_to_i16(x: u16) -> i16 {
    unsafe {
        core::mem::transmute(x)
    }
}

