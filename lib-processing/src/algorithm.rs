use core::f32::consts::PI;
use math::*;
use crate::*;
use cfg_if::cfg_if;

pub struct ProcessingAlgorithm
{
    /// In nanoseconds; timestamp of the previously processed data packet.
    pub prev_time: f32,

    /// Complementary filter mixing factor.
    pub alpha: f32,

    /// The current orientation of the device
    pub orientation: Quaternion,

    /// In m/s; The current velocity vector of the device.
    pub velocity: Vector,

    /// In meters; The current position in space of the device.
    pub position: Vector,

    /// Computed linear acceleration for the current packet, i.e. the world acceleration without
    /// gravity included.
    #[cfg(feature = "debug")]
    pub world_acceleration: Vector,

    /// Orientation according only to integrating gyroscope, ignoring accelerometer.
    #[cfg(feature = "debug")]
    pub gyro_orientation: Quaternion,

    /// Orientation according only to accelerometer's gravity vector, ignoring gyroscope.
    #[cfg(feature = "debug")]
    pub accel_orientation: Quaternion,
}


impl ProcessingAlgorithm {

    #[inline]
    pub fn new() -> Self {
        ProcessingAlgorithm {
            prev_time: 0.0,
            alpha: 0.1,
            orientation: Quaternion::from(EulerAngles::new(0.0, 0.0, PI / -2.0)),
            velocity: Vector::zero(),
            position: Vector::zero(),
        }
    }

    ///
    pub fn step(&mut self, time: f32, accel: Vector, gyro: Vector) 
    {
        // Skip the first packet since `prev_time` is not initialized yet.
        if self.prev_time < 0.001 {
            self.prev_time = time;
            return;
        }

        // How much time has passed since the previous packet?
        let delta_t = time - self.prev_time;
        self.prev_time = time;

        // How much has the rotation changed according to the gyroscope?
        let theta = gyro.magnitude() * delta_t;
        let mut q_gyro_delta = Quaternion::identity();
        if theta > 0.0 {
            let axis = gyro.normalize();
            q_gyro_delta = Quaternion::new(
                libm::cosf(theta / 2.0),
                axis.x * libm::sinf(theta / 2.0),
                axis.y * libm::sinf(theta / 2.0),
                axis.z * libm::sinf(theta / 2.0),
            );
        }
        
        // Add the computed change to the current orientation to get the new device orientation
        // according to the gyroscope
        let q_gyro = (self.orientation * q_gyro_delta).normalize();
        
        cfg_if!{ if #[cfg(feature = "debug")] {
            self.gyro_orientation = q_gyro;
        }}

        // Compute the new device orientation according to the accelerometer based on the 
        // direction of gravity.
        let mut euler_accel = EulerAngles::from(quaternion_from_accel(&accel));
        euler_accel.yaw = 0.0; // Gravity cannot give us yaw value.
        let q_accel = Quaternion::from(euler_accel);
    
        cfg_if!{ if #[cfg(feature = "debug")] {
            self.accel_orientation = q_accel;
        }}

        // Apply a complimentary filter to both readings to maintain long term stability.
        self.orientation = complementary_filter(&q_gyro, &q_accel, self.alpha);

        // Rotate the acceleration vector into world space
        let accel_world = self.orientation.rotate(&accel);

        // Subtract gravity to only leave linear acceleration
        let accel_world_corrected = accel_world - (REFERENCE_GRAVITY * G_TO_MS2);
        
        cfg_if!{ if #[cfg(feature = "debug")] {
            self.world_acceleration = accel_world_corrected;
        }}

        // Integrate acceleration to update velocity
        self.velocity *= 0.9; // Reduce acceleration a little to keep it from spiraling out of control.
        self.velocity += accel_world_corrected * delta_t;

        // Integrate velocity to update position
        self.position += self.velocity * delta_t;
    }
}

