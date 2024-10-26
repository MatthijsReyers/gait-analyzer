
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum GyroScaleRange
{
    D250 = 0,
    D500 = 1,
    D1000 = 2,
    D2000 = 3,
}

impl GyroScaleRange {
    
    /// Converts the given full scale range setting into the bits one would need to write into the
    /// `GYRO_CONFIG` register to configure the sensor to use that scale range.
    /// 
    pub fn as_register(&self) -> u8 {
        ((*self) as u8) << 3
    }
    
    /// Gets the full scale range currently configured in the `GYRO_CONFIG` register based on its
    /// contents.
    /// 
    pub fn from_register(value: u8) -> Self {
        // This is safe because the bitwise `&` ensures that the value <= 3.
        unsafe {
            core::mem::transmute((value >> 3) & 0b011)
        }
    }
    
    /// Gets the sensitivity scale factor for the given scale range.
    /// (Note scale factor is in LSB / (deg/s)).
    /// 
    pub fn as_scale_factor(&self) -> f32 {
        match self {
            Self::D250 => 131.0,
            Self::D500 => 65.5,
            Self::D1000 => 32.8,
            Self::D2000 => 16.4,
        }
    }
}

impl Default for GyroScaleRange {
    fn default() -> Self {
        GyroScaleRange::D250
    }
}
