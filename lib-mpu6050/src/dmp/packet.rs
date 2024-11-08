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

impl DMPPacket
{
    /// Uses the device orientation stored in the Quaternion computed by the DMP to compute a 
    /// unit vector in the direction of world gravity. (In order words: this function returns a 
    /// vector pointing towards the earth).
    /// 
    pub fn get_gravity(&self) -> Vector {
        Vector {
            x: 2.0 * (self.quaternion.x * self.quaternion.z - self.quaternion.w * self.quaternion.y),
            y: 2.0 * (self.quaternion.w * self.quaternion.x + self.quaternion.y * self.quaternion.z),
            z: (self.quaternion.w * self.quaternion.w) - (self.quaternion.x * self.quaternion.x) 
                - (self.quaternion.x * self.quaternion.x) + (self.quaternion.z * self.quaternion.z)
        }
    }
}
