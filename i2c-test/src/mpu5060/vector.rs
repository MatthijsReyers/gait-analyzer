use core::ops::{Add, Div, DivAssign, Mul, MulAssign, Sub};

#[derive(Debug, Clone, Copy)]
pub struct Vector
{
    pub x: f32, 
    pub y: f32,
    pub z: f32,
}

impl From<[f32; 3]> for Vector {
    fn from(values: [f32; 3]) -> Self {
        Self {
            x: values[0],
            y: values[1],
            z: values[2],
        }
    }
}

impl Vector 
{
    /// Returns a zero vector.
    /// 
    pub fn zero() -> Self {
        Vector { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Calculate the length of the vector
    /// 
    pub fn length(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize the vector
    /// 
    pub fn normalize(&self) -> Vector {
        let len = self.length();
        if len == 0.0 {
            // Avoid division by zero; return a zero vector
            return Vector { x: 0.0, y: 0.0, z: 0.0 };
        }
        Vector {
            x: self.x / len,
            y: self.y / len,
            z: self.z / len,
        }
    }
}

impl Div for Vector
{
    type Output = Self;

    fn div(self, other: Self) -> Self::Output {
        Vector {
            x: self.x / other.x,
            y: self.y / other.y,
            z: self.z / other.z,
        }
    }
}

impl Add for Vector
{
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Vector {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vector
{
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        self + (other * -1)
    }
}

impl Mul<Vector> for Vector
{
    type Output = Self;

    fn mul(self, other: Self) -> Self::Output {
        Vector {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }
}

impl Div<f32> for Vector
{
    type Output = Self;

    fn div(self, other: f32) -> Self::Output {
        Vector {
            x: self.x / other,
            y: self.y / other,
            z: self.z / other,
        }
    }
}

impl Div<i32> for Vector
{
    type Output = Self;

    fn div(self, other: i32) -> Self::Output {
        self / (other as f32)
    }
}

impl DivAssign<f32> for Vector
{
    fn div_assign(&mut self, other: f32) {
        self.x /= other;
        self.y /= other;
        self.z /= other;
    }
}

impl Mul<f32> for Vector
{
    type Output = Self;

    fn mul(self, other: f32) -> Self::Output {
        Vector {
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
        }
    }
}

impl Mul<i32> for Vector
{
    type Output = Self;

    fn mul(self, other: i32) -> Self::Output {
        self * (other as f32)
    }
}

impl MulAssign<Vector> for Vector
{
    fn mul_assign(&mut self, other: Self) {
        self.x *= other.x;
        self.y *= other.y;
        self.z *= other.z;
    }
}

impl MulAssign<f32> for Vector
{
    fn mul_assign(&mut self, other: f32) {
        self.x *= other;
        self.y *= other;
        self.z *= other;
    }
}
