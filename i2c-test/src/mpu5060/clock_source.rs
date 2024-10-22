
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum ClockSource
{
    InternalOscillator = 0,

    GyroX = 1,
    GyroY = 2,
    GyroZ = 3,
    
    External32kHz = 4,
    External19MHz = 5,

    /// DO NOT USE! Undefined behaviour
    // Reserved = 6,

    /// Stops the clock and keeps the timing generator in reset
    Stop = 7,
}
