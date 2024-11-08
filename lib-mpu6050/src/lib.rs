#![no_std]

#[cfg(not(any(feature = "dmp612", feature = "dmp20")))]
compile_error!("No DMP firmware version configured!");

use math::*;

pub mod accel_scale_range;
pub use accel_scale_range::*;

pub mod gyro_scale_range;
pub use gyro_scale_range::*;

pub mod data;
pub use data::*;

pub mod i2c_slave;
pub use i2c_slave::*;

pub mod clock_source;
pub use clock_source::*;

pub mod dlpf_mode;
pub use dlpf_mode::*;

pub mod registers;

#[cfg(feature = "hal")]
pub mod mpu6050;
#[cfg(feature = "hal")]
pub use mpu6050::*;

mod utils;

pub mod dmp;

/// Default i2c address of the MPU 6050 chip.
/// 
pub const MPU6505_DEFAULT_I2C_ADDR: u8 = 0x68;

/// The default device ID of a MPU6050 chip.
/// 
pub const MPU6050_DEVICE_ID: u8 = 0x034;


// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }
