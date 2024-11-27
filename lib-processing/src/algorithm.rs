use core::f32::consts::PI;
use math::*;
use crate::*;
use cfg_if::cfg_if;

pub struct ProcessingAlgorithm
{
    /// In nanoseconds; timestamp of the previously processed data packet.
    pub prev_time: i64,

    /// Complementary filter mixing factor.
    pub alpha: f32,

    /// The current orientation of the device
    pub orientation: Quaternion,

    /// In meters; The current position of the device in world space, i.e. with the starting/power
    /// on position as the origin.
    pub position: Vector,

    /// In m/s; The current velocity vector of the device.
    pub velocity: Vector,

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
            prev_time: 0,
            alpha: 0.05,
            orientation: Quaternion::identity(),
            velocity: Vector::zero(),
            position: Vector::zero(),

            #[cfg(feature = "debug")]
            world_acceleration: Vector::zero(),
            #[cfg(feature = "debug")]
            gyro_orientation: Quaternion::identity(),
            #[cfg(feature = "debug")]
            accel_orientation: Quaternion::identity(),
        }
    }

    /// Compute one time step of the algorithm
    /// 
    pub fn step(&mut self, time: i64, accel: Vector, gyro: Vector) 
    {
        // Skip the first packet since `prev_time` is not initialized yet.
        if self.prev_time == 0 {
            self.prev_time = time;
            return;
        }

        // How much time has passed since the previous packet?
        let delta_t = ((time - self.prev_time) as f32) / 1_000_000_000.0;
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
        self.velocity *= 0.95; // Reduce velocity a little to keep it from spiraling out of control.
        self.velocity += accel_world_corrected * delta_t;

        // Integrate velocity to update position
        self.position += self.velocity * delta_t;
    }

    #[cfg(feature = "csv")]
    pub fn get_csv_header(&self) -> String {
        "time,".to_string()
        + "orientation.w,orientation.x,orientation.y,orientation.z,"
        + "velocity.x,velocity.y,velocity.z,"
        + "position.x,position.y,position.z,"
        + "world_acc.x,world_acc.y,world_acc.z,"
        + "gyro_orientation.w,gyro_orientation.x,gyro_orientation.y,gyro_orientation.z,"
        + "accel_orientation.w,accel_orientation.x,accel_orientation.y,accel_orientation.z\n"
    }

    #[cfg(feature = "csv")]
    pub fn get_csv_state(&self) -> String {
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            self.prev_time,
            self.orientation.w,
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z,
            self.position.x,
            self.position.y,
            self.position.z,
            self.world_acceleration.x,
            self.world_acceleration.y,
            self.world_acceleration.z,
            self.gyro_orientation.w,
            self.gyro_orientation.x,
            self.gyro_orientation.y,
            self.gyro_orientation.z,
            self.accel_orientation.w,
            self.accel_orientation.x,
            self.accel_orientation.y,
            self.accel_orientation.z,
        )
    }
}

