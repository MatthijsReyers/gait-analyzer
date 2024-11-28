use core::ops::{Div, Mul};
use crate::*;

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

    /// Clone/copy in place option to replace the current values with those of the other quaternion
    /// 
    #[inline]
    pub fn replace(&mut self, other: &Quaternion) {
        self.x = other.x;
        self.y = other.y;
        self.z = other.z;
        self.w = other.w;
    }

    /// Get the magnitude of the quaternion.
    /// 
    #[inline]
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize the quaternion to make it a unit quaternion.
    /// 
    pub fn normalize(&self) -> Quaternion {
        let magnitude_2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        // Cannot normalize a quaternion with zero magnitude.
        if magnitude_2 == 0.0 {
            return self.clone();
        }
        let magnitude = libm::sqrtf(magnitude_2);
        Quaternion {
            w: self.w / magnitude,
            x: self.x / magnitude,
            y: self.y / magnitude,
            z: self.z / magnitude,
        }
    }

    /// Compute the conjugate of the quaternion.
    /// 
    pub fn conjugate(&self) -> Self {
        Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn rotate(&self, vector: &Vector) -> Vector {
        // Convert the vector into a quaternion with w = 0
        let vector_quat = Quaternion::new(0.0, vector.x, vector.y, vector.z);

        // Rotate the vector using q * v * q^-1
        let rotated_quat = self * vector_quat * self.conjugate();

        // Extract the rotated vector part (x, y, z)
        Vector {
            x: rotated_quat.x,
            y: rotated_quat.y,
            z: rotated_quat.z,
        }
    }

    /// Approximate equality check with a given tolerance.
    /// 
    pub fn approx_eq(&self, other: &Quaternion, tol: f32) -> bool {
        libm::fabsf(self.x - other.x) <= tol
            && libm::fabsf(self.y - other.y) <= tol
            && libm::fabsf(self.z - other.z) <= tol
            && libm::fabsf(self.w - other.w) <= tol
    }

    /// Quaternion multiplication.
    /// 
    pub fn multiply(&self, other: &Quaternion) -> Quaternion {
        Quaternion {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }
}

impl Mul<Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, other: Quaternion) -> Self::Output {
        self.multiply(&other)
    }
}
impl Mul<&Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, other: &Quaternion) -> Self::Output {
        self.multiply(other)
    }
}
impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;
    fn mul(self, other: Quaternion) -> Self::Output {
        (&self).multiply(&other)
    }
}
impl Mul<&Quaternion> for Quaternion {
    type Output = Quaternion;
    fn mul(self, other: &Quaternion) -> Self::Output {
        (&self).multiply(other)
    }
}

impl From<&EulerAngles> for Quaternion {
    fn from(a: &EulerAngles) -> Self {
        let cy = libm::cosf(a.yaw * 0.5);
        let sy = libm::sinf(a.yaw * 0.5);
        let cp = libm::cosf(a.pitch * 0.5);
        let sp = libm::sinf(a.pitch * 0.5);
        let cr = libm::cosf(a.roll * 0.5);
        let sr = libm::sinf(a.roll * 0.5);

        Quaternion {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
}

impl From<EulerAngles> for Quaternion {
    fn from(angles: EulerAngles) -> Self {
        Quaternion::from(&angles)
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
