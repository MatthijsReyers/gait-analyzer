use core::{f32::NAN, ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign}};

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
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Vector { x, y, z }
    }

    /// Returns a zero vector.
    /// 
    pub const fn zero() -> Self {
        Vector { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Calculate the length/magnitude of the vector
    /// 
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize the vector
    /// 
    pub fn normalize(&self) -> Vector {
        let len = self.magnitude();
        if len == 0.0 || len == NAN {
            // Avoid division by zero; return a zero vector
            return Vector::zero();
        }
        self / len
    }

    /// Take the dot product of two vectors.
    /// 
    pub fn dot(&self, other: &Vector) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Get the cross product of two vectors.
    /// 
    pub fn cross(&self, other: &Vector) -> Vector {
        Vector {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Approximate equality check with a given tolerance.
    pub fn approx_eq(&self, other: &Vector, tol: f32) -> bool {
        libm::fabsf(self.x - other.x) <= tol
            && libm::fabsf(self.y - other.y) <= tol
            && libm::fabsf(self.z - other.z) <= tol
    }

    /// Replaces the value with the given vector, this essentially clones the other vector in place
    /// 
    #[inline]
    pub fn replace(&mut self, other: &Vector) {
        self.x = other.x;
        self.y = other.y;
        self.z = other.z;
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

impl AddAssign for Vector
{
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl Sub for Vector
{
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Vector {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl SubAssign for Vector
{
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
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

impl Div<f32> for &Vector
{
    type Output = Vector;

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

impl MulAssign<f32> for Vector
{
    fn mul_assign(&mut self, other: f32) {
        self.x *= other;
        self.y *= other;
        self.z *= other;
    }
}

impl MulAssign<i32> for Vector
{
    fn mul_assign(&mut self, other: i32) {
        self.mul_assign(other as f32);
    }
}
