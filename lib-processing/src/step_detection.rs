use crate::*;
use cfg_if::cfg_if;

static STEP_START_THRESHOLD: f32 = 0.05;

/// In meters, default/reset value for the peak gate, since it cannot start at 0 since it would
/// instantly trigger and end the step.
/// 
static PEAK_GATE_DEFAULT: f32 = 0.08; // == 8cm

pub struct StepDetection
{
    /// Have we started detecting a step yet?
    /// 
    pub step_started: Option<i64>,
    
    /// Has the step peaked? I.e. has the position reached it's max height.
    /// 
    pub step_peaked: bool,

    /// Sensor position at the peak of the movement.
    /// 
    pub peak_pos: Vector,
    
    /// In cm, height at which we consider the step "peaked", the code always sets this value to
    /// max_offset.x / 2, this value only exists to avoid doing that division every loop since we
    /// only have to update this value when max_offset changes.
    /// 
    pub peak_gate: f32,

    /// Have we detected a full step? I.e. should the user copy the data they want and should we
    /// reset the algorithm state next step?
    /// If so the value is the timestamp at which the algorithm detected the step as being 
    /// finished.
    /// 
    pub step_finished: Option<i64>,

    /// Position at which the step detection was finished.
    /// 
    pub finished_pos: Vector,

    /// Orientation at the end of the step movement, practically speaking this should roughly be
    /// the is the angle of the hoof when it hits the ground.
    /// 
    pub finished_orientation: Quaternion,


    #[cfg(feature = "debug")]
    pub step_peak: Option<i64>,

    #[cfg(feature = "debug")]
    pub step_peaked_trigger: Option<i64>,
}

impl StepDetection
{
    #[inline]
    pub fn new() -> Self {
        StepDetection {
            step_started: None,
            step_peaked: false,
            step_finished: None, 

            peak_pos: Vector::zero(),
            peak_gate: PEAK_GATE_DEFAULT,

            finished_pos: Vector::zero(),
            finished_orientation: Quaternion::identity(),

            #[cfg(feature = "debug")]
            step_peak: None,
            #[cfg(feature = "debug")]
            step_peaked_trigger: None,
        }
    }

    pub fn reset(&mut self) {
        self.step_peaked = false;
        self.step_finished = None;
        self.step_started = None;
        
        self.peak_pos = Vector::zero();
        self.peak_gate = PEAK_GATE_DEFAULT;
        
        self.finished_pos = Vector::zero();
        self.finished_orientation = Quaternion::identity();
    
        cfg_if!{ if #[cfg(feature = "debug")] {
            self.step_peak = None;
            self.step_peaked_trigger = None;
        }}
    }

    /// Compute one time step of the algorithm
    /// 
    pub fn step(&mut self, sensor: &mut SensorFusion) {

        if self.step_finished.is_some() {
            sensor.position = Vector::zero();
            self.reset();
        }

        if self.step_started.is_none() && sensor.position.z > STEP_START_THRESHOLD {
            self.step_started = Some(sensor.prev_time);
            log::debug!("Step started: {}", sensor.prev_time);
        }

        if self.step_started.is_some() && !self.step_peaked {
            if sensor.position.z > self.peak_pos.z {
                self.peak_pos.replace(&sensor.position);
                self.peak_gate = self.peak_pos.z / 2.0;
                cfg_if!{ if #[cfg(feature = "debug")] {
                    self.step_peak = Some(sensor.prev_time);
                }}
            }
        
            if sensor.position.z < self.peak_gate {
                self.step_peaked = true;
                self.finished_orientation.replace(&sensor.orientation);

                cfg_if!{ if #[cfg(feature = "debug")] {
                    self.step_peaked_trigger = Some(sensor.prev_time);
                }}
            }
        }

        if self.step_peaked && self.step_finished.is_none() {
            if sensor.velocity.magnitude() < 0.05 {
                self.step_finished = Some(sensor.prev_time);
                self.finished_pos.replace(&sensor.position);
            }
        }
    }
}