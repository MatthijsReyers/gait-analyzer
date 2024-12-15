use crate::*;
// use cfg_if::cfg_if;

// static STEP_START_THRESHOLD: f32 = 0.05;

// /// In meters, default/reset value for the peak gate, since it cannot start at 0 since it would
// /// instantly trigger and end the step.
// /// 
// static PEAK_GATE_DEFAULT: f32 = 0.08; // == 8cm

/// In nanoseconds, the maximum time that a step is allowed to take, after this time interval we 
/// assume that the step is finished regardless of what data is coming in. (Note that this is to 
/// ensure that the algorithm can never get stuck in some half invalid state).
/// 
static MAX_STEP_DURATION: i64 = 5e9 as i64; // == 0.5 second


#[derive(Debug, Default)]
pub struct StepDetection
{
    pub start_time: Option<i64>,
    pub start_position: Option<Vector>,
    pub start_velocity: Option<Vector>,
    pub start_orientation: Option<Quaternion>,

    pub peak_time: i64,
    pub peak_position: Vector,
    pub peak_velocity: Vector,
    pub peak_passed: bool,

    pub stop_time: Option<i64>,
    pub stop_min_velocity: Vector,

    pub last_step: Option<i64>,

    // /// Have we started detecting a step yet?
    // /// 
    // pub step_started: Option<i64>,
    
    // /// Has the step peaked? I.e. has the position reached it's max height.
    // /// 
    // pub step_peaked: bool,

    // /// Sensor position at the peak of the movement.
    // /// 
    // pub peak_pos: Vector,
    
    // /// In cm, height at which we consider the step "peaked", the code always sets this value to
    // /// max_offset.x / 2, this value only exists to avoid doing that division every loop since we
    // /// only have to update this value when max_offset changes.
    // /// 
    // pub peak_gate_velocity: f32,

    // /// Have we detected a full step? I.e. should the user copy the data they want and should we
    // /// reset the algorithm state next step?
    // /// If so the value is the timestamp at which the algorithm detected the step as being 
    // /// finished.
    // /// 
    // pub step_finished: Option<i64>,

    // /// Position at which the step detection was finished.
    // /// 
    // pub finished_pos: Vector,

    // /// Orientation at the end of the step movement, practically speaking this should roughly be
    // /// the is the angle of the hoof when it hits the ground.
    // /// 
    // pub finished_orientation: Quaternion,

    // #[cfg(feature = "debug")]
    // pub step_peak: Option<i64>,

    // #[cfg(feature = "debug")]
    // pub step_peaked_trigger: Option<i64>,
}


impl StepDetection
{
    #[inline]
    pub fn new() -> Self {
        StepDetection {
            ..Default::default()
            // step_started: None,
            // step_peaked: false,
            // step_finished: None, 

            // peak_pos: Vector::zero(),
            // // peak_gate: PEAK_GATE_DEFAULT,

            // finished_pos: Vector::zero(),
            // finished_orientation: Quaternion::identity(),

            // #[cfg(feature = "debug")]
            // step_peak: None,
            // #[cfg(feature = "debug")]
            // step_peaked_trigger: None,
        }
    }

    #[inline]
    pub fn step_has_started(&self) -> bool {
        self.start_time.is_some()
    } 

    #[inline]
    pub fn step_is_done(&self) -> bool {
        self.stop_time.is_some()
    } 

    pub fn reset(&mut self) {
        // self.step_peaked = false;
        // self.step_finished = None;
        // self.step_started = None;
        
        // self.peak_pos = Vector::zero();
        // // self.peak_gate = PEAK_GATE_DEFAULT;
        
        // self.finished_pos = Vector::zero();
        // self.finished_orientation = Quaternion::identity();
    
        // cfg_if!{ if #[cfg(feature = "debug")] {
        //     self.step_peak = None;
        //     self.step_peaked_trigger = None;
        // }}

        self.start_time = None;
        self.start_position = None;
        self.start_velocity = None;
        self.peak_position = Vector::zero();
        self.peak_velocity = Vector::zero();
        self.peak_time = 0;
        self.peak_passed = false;
        self.stop_min_velocity = Vector::zero();
        self.stop_time = None;
    }

    /// Compute one time step of the algorithm
    /// 
    pub fn update(&mut self, sensor: &mut SensorFusion, accel: &Vector) {
        
        if let Some(stop_t) = self.stop_time {
            self.last_step = Some(stop_t);
            self.reset();
            return;
        }

        if let Some(last_step) = self.last_step {
            if sensor.prev_time > last_step + (8e7 as i64) {
                sensor.velocity = Vector::zero();
                sensor.position = Vector::zero();
                log::warn!("Reset position and velocity after last step: {}", sensor.prev_time);
                self.last_step = None;
            }
        }

        match self.start_time {
            // Step has not started yet.
            None => {
                let euler = EulerAngles::from(&sensor.orientation);

                if euler.pitch > 0.0 && (accel.magnitude() - 9.8) > 15.0 {

                    self.start_time = Some(sensor.prev_time);
                    self.start_position = Some(sensor.position);
                    self.start_velocity = Some(sensor.velocity);
                    self.start_orientation = Some(sensor.orientation);
    
                    log::debug!("Step started: {}", sensor.prev_time);
                }
            },
            // Step has started.
            Some(start_t) => {
                if self.peak_velocity.sum() < sensor.velocity.sum() {
                    self.peak_velocity = sensor.velocity;
                }
    
                // Only look at Z-axis since horse is probably moving forward and other axis should
                // thus always be bigger next measurement.
                if self.peak_position.z < sensor.position.z {
                    self.peak_position = sensor.position;
                    self.peak_time = sensor.prev_time;
                }
    
                // CHECK: has the max step duration passed?
                if sensor.prev_time - start_t > MAX_STEP_DURATION {
                    log::error!("Reset because of step timeout");
                    log::debug!("Peak passed: {}", sensor.prev_time);
                    log::debug!("Minimum velocity sum: {}", self.stop_min_velocity.sum());
                    self.reset();
                }
    
                if !self.peak_passed && /* self.peak_velocity.magnitude() > 4.0 && */ self.peak_velocity.sum() / 3.0 > sensor.velocity.sum() {
                    self.peak_passed = true;
                    self.stop_min_velocity = self.peak_velocity;
                    log::debug!("Peak passed: {}", sensor.prev_time);
                    return;
                }

                if self.peak_passed {
                    if sensor.velocity.sum() < self.stop_min_velocity.sum() {
                        self.stop_min_velocity = sensor.velocity;
                        return;
                    }

                    if sensor.velocity.sum() > self.stop_min_velocity.sum() + 0.2 {
                        log::debug!("Minimum velocity sum: {}", self.stop_min_velocity.sum());
                        log::debug!("Step ended: {}", sensor.prev_time);
                        self.stop_time = Some(sensor.prev_time);
                    }
                }


                // if self.peak_passed && (accel.magnitude() - 9.8) < 20.0 {
                //     log::debug!("Step ended: {}", sensor.prev_time);
                //     self.stop_time = Some(sensor.prev_time)
                // }
            }
        }
    }
}