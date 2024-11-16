use core::ops::{Div, Mul};

use crate::Vector;


#[derive(Debug, Clone, Copy)]
pub struct Quaternion
{
    pub w: f32,
    pub x: f32, 
    pub y: f32,
    pub z: f32,
}

impl From<[f32; 4]> for Quaternion {
    fn from(values: [f32; 4]) -> Self {
        Self {
            w: values[0],
            x: values[1],
            y: values[2],
            z: values[3],
        }
    }
}


impl Quaternion 
{
    /// Create a new quaternion with the given values.
    /// 
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Quaternion { w, x, y, z }
    }

    /// Returns the identity quaternion (no rotation)
    /// 
    pub fn identity() -> Self {
        Quaternion {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
    
    /// Constructs a quaternion from a scalar (w) and a vector (x, y, z)
    /// 
    pub fn from_parts(w: f32, vector: Vector) -> Self {
        Quaternion {
            w,
            x: vector.x,
            y: vector.y,
            z: vector.z,
        }
    }

    /// Normalize the quaternion to make it a unit quaternion.
    /// 
    pub fn normalize(&self) -> Quaternion {
        let magnitude_2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        // Cannot normalize a quaternion with zero magnitude.
        if magnitude_2 == 0.0 {
            return *self;
        }
        let magnitude = libm::sqrtf(magnitude_2);
        Quaternion {
            w: self.w / magnitude,
            x: self.x / magnitude,
            y: self.y / magnitude,
            z: self.z / magnitude,
        }
    }
}


impl Div<f32> for Quaternion
{
    type Output = Self;

    fn div(self, other: f32) -> Self::Output {
        Quaternion {
            w: self.w / other,
            x: self.x / other,
            y: self.y / other,
            z: self.z / other,
        }
    }
}

impl Mul<f32> for Quaternion
{
    type Output = Self;

    fn mul(self, other: f32) -> Self::Output {
        Quaternion {
            w: self.w * other,
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
        }
    }
}
