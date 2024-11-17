#![cfg_attr(not(test), no_std)]

pub mod float_utils;
pub use float_utils::*;

pub mod euler_angles;
pub use euler_angles::*;

pub mod vector;
pub use vector::*;

pub mod quaternion;
pub use quaternion::*;

#[cfg(test)]
mod tests;

pub const G_TO_MS2: f32 = 9.80665;

pub const DEG_TO_RAD: f32 = 0.0174533;

pub const RAD_TO_DEG: f32 = 57.29578;
