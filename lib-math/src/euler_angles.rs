use crate::*;

/// Orientation as a product of three rotations, note that Euler angles suffer from gimbal lock and
/// we thus should avoid storing/processing orientation in Euler angles. (Although they are useful
/// for displaying orientation since they are much easier to understand for humans).
/// 
/// Yaw = rotation around Z-axis
/// Roll = rotation around X-axis
/// Pitch = rotation around Y-axis
/// 
#[derive(Debug, Clone, Copy)]
pub struct EulerAngles
{
    pub yaw: f32,
    pub pitch: f32, 
    pub roll: f32,
}

impl EulerAngles
{
    pub const fn new(yaw: f32, pitch: f32, roll: f32) -> Self {
        EulerAngles { yaw, pitch, roll }
    }

    /// Creates an all zeros euler angles instance, i.e. the identity/no rotation angles.
    /// 
    pub const fn identity() -> Self {
        EulerAngles { yaw: 0.0, pitch: 0.0, roll: 0.0 }
    }

    /// Approximate equality check with a given tolerance.
    pub fn approx_eq(&self, other: &Self, tol: f32) -> bool {
        libm::fabsf(self.yaw - other.yaw) <= tol
            && libm::fabsf(self.pitch - other.pitch) <= tol
            && libm::fabsf(self.roll - other.roll) <= tol
    }
}

impl From<Quaternion> for EulerAngles {
    fn from(value: Quaternion) -> Self {
        EulerAngles::from(&value)
    }
}

impl From<&Quaternion> for EulerAngles {
    fn from(q: &Quaternion) -> Self {
        let ysqr = q.y * q.y;

        // Roll (x-axis rotation)
        let t0 = 2.0 * (q.w * q.x + q.y * q.z);
        let t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
        let roll = libm::atan2f(t0, t1);

        // Pitch (y-axis rotation)
        let t2 = 2.0 * (q.w * q.y - q.z * q.x);
        // Clamp to avoid invalid domain
        let t2_clamped = if t2 > 1.0 { 1.0 } else if t2 < -1.0 { -1.0 } else { t2 }; 
        let pitch = libm::asinf(t2_clamped);

        // Detect gimbal lock
        if libm::fabsf(t2_clamped) >= 0.999999 {
            let yaw = 0.0; // Arbitrarily set yaw to 0
            let roll = libm::atan2f(-2.0 * (q.y * q.z - q.w * q.x), 1.0 - 2.0 * (q.x * q.x + q.z * q.z));
            return EulerAngles::new(yaw, pitch, roll);
        }

        // Yaw (z-axis rotation)
        let t3 = 2.0 * (q.w * q.z + q.x * q.y);
        let t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
        let yaw = libm::atan2f(t3, t4);

        EulerAngles::new(yaw, pitch, roll)
    }
}

impl From<Vector> for EulerAngles {
    
    /// Create Euler angles from the given rotations around each axis of the vector. Please note
    /// that this conversion only makes sense if the vector contains rotations in radians! You need
    /// other utilities for creating rotations around a vector or things like that.
    /// 
    /// Also remember the meaning of the euler angles and note that order in which we talk about 
    /// euler angles is not X,Y,Z, but Z,X,Y since:
    /// 
    /// Yaw = rotation around Z-axis
    /// Roll = rotation around X-axis
    /// Pitch = rotation around Y-axis
    /// 
    #[inline]
    fn from(v: Vector) -> Self {
        EulerAngles::new(
            v.z, 
            v.y, 
            v.x
        )
    }
}
