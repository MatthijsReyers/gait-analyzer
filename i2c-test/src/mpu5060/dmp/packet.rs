use crate::Vector;


pub const DMP_PACKET_SIZE: u16 = 42;

#[derive(Debug, Clone, Copy)]
pub struct DMPPacket {
    pub accel: Vector,
    pub gyro: Vector,
}
