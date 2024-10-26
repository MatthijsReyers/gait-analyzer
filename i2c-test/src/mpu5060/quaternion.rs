use core::ops::{Div, Mul};


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
    /// Returns a zero vector.
    /// 
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Quaternion { w, x, y, z }
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
