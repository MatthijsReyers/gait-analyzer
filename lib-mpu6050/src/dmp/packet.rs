use crate::{Quaternion, Vector};

#[cfg(feature = "dmp20")]
pub const DMP_PACKET_SIZE: u16 = 42;

#[cfg(feature = "dmp612")]
pub const DMP_PACKET_SIZE: u16 = 28;

#[cfg(not(any(feature = "dmp612", feature = "dmp20")))]
pub const DMP_PACKET_SIZE: u16 = 0;

#[derive(Debug, Clone, Copy)]
pub struct DMPPacket {
    pub accel: Vector,
    pub gyro: Vector,
    pub quaternion: Quaternion,
}
