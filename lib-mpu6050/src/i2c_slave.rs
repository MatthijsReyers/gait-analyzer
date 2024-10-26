
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cSlave
{
    Slave0 = 0,
    Slave1 = 1,
    Slave2 = 2,
    Slave3 = 3,
}

