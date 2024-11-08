
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum AccelScaleRange
{
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelScaleRange {
    
    /// Converts the given full scale range setting into the bits one would need to write into the
    /// `ACCEL_CONFIG` register to configure the sensor to use that scale range.
    /// 
    pub fn as_register(&self) -> u8 {
        ((*self) as u8) << 3
    }
    
    /// Gets the full scale range currently configured in the `ACCEL_CONFIG` register based on its
    /// contents.
    /// 
    pub fn from_register(value: u8) -> Self {
        // This is safe because the bitwise and ensures that the value <= 3.
        unsafe {
            core::mem::transmute((value >> 3) & 0b011)
        }
    }
    
    /// Gets the sensitivity scale factor for the given scale range.
    /// (Note scale factor is in LSB/g).
    /// 
    pub fn as_scale_factor(&self) -> f32 {
        match self {
            Self::G2 => 16384.0,    // Fixed point between 2-3 MSB bits.
            Self::G4 => 8192.0,     // Fixed point between 3-4 MSB bits.
            Self::G8 => 4096.0,     // Fixed point between 4-5 MSB bits.
            Self::G16 => 2048.0,    // Fixed point between 5-6 MSB bits.
        }
    }
}

impl Default for AccelScaleRange {
    fn default() -> Self {
        AccelScaleRange::G2
    }
}