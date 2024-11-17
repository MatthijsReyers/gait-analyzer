use math::*;
use crate::*;

/// Blends two quaternions using a complementary filter.
/// 
pub fn complementary_filter(q_gyro: &Quaternion, q_accel: &Quaternion, alpha: f32) -> Quaternion {
    let beta = 1.0 - alpha;
    Quaternion {
        w: alpha * q_accel.w + beta * q_gyro.w,
        x: alpha * q_accel.x + beta * q_gyro.x,
        y: alpha * q_accel.y + beta * q_gyro.y,
        z: alpha * q_accel.z + beta * q_gyro.z,
    }
    .normalize() // Ensure the resulting quaternion is a unit quaternion
}

 /// Tries to compute the current device rotation (in quaternion form) based on the direction 
/// of gravity measured by the accelerometer. Note that this code does nothing to account for
/// the other accelerations the device experiences so the computed rotation is not very 
/// accurate in short intervals or when the device is moving quickly.
/// 
pub fn quaternion_from_accel(accel: &Vector) -> Quaternion
{
    // TODO: The computed roll/pitch are inverted for some reason if I don't do this, we should
    // probably figure out why that is?
    let mut accel = accel.clone();
    accel.y *= -1.0;
    accel.x *= -1.0;
    
    // Gravity vector from the accelerometer in the local/device space.
    let gravity = accel.normalize();

    // Compute the cross product (axis of rotation)
    let v = REFERENCE_GRAVITY.cross(&gravity);

    // Compute the dot product (cosine of the angle)
    let cos_theta = REFERENCE_GRAVITY.dot(&gravity);

    // Compute the angle (theta) using the dot product
    let theta = libm::acosf(cos_theta);

    // Handle edge cases
    if v.magnitude() == 0.0 {
        return Quaternion::identity(); 
    }

    // Normalize the rotation axis
    let axis = v.normalize();

    // Compute the quaternion
    let half_theta = theta / 2.0;
    let sin_half_theta = libm::sinf(half_theta);

    Quaternion::from_parts(
        libm::cosf(half_theta),     // w (scalar part)
        axis * sin_half_theta, // x, y, z (vector part)
    )
}
