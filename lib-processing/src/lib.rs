#![cfg_attr(not(feature = "csv"), no_std)]

use math::*;

pub mod utils;
pub use utils::*;

pub mod sensor_fusion;
pub use sensor_fusion::*;

pub mod step_detection;
pub use step_detection::*;

/// Gravity acceleration reference vector, i.e. the opposite direction of gravity in world space.
///
pub static REFERENCE_GRAVITY: Vector = Vector::new(0.0, 0.0, 1.0);

#[cfg(test)]
pub mod tests;
