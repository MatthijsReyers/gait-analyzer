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
    pub step_started: bool,
    
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
    

    #[cfg(feature = "debug")]
    pub step_start: Option<i64>,

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
            step_started: false,
            peak_pos: Vector::zero(),
            peak_gate: PEAK_GATE_DEFAULT,
            step_peaked: false,

            #[cfg(feature = "debug")]
            step_start: None,
            #[cfg(feature = "debug")]
            step_peak: None,
            #[cfg(feature = "debug")]
            step_peaked_trigger: None,
        }
    }

    pub fn reset(&mut self) {
        self.step_started = false;
        self.peak_pos = Vector::zero();
        self.peak_gate = PEAK_GATE_DEFAULT;
        self.step_peaked = false;
    
        cfg_if!{ if #[cfg(feature = "debug")] {
            self.step_start = None;
            self.step_peak = None;
            self.step_peaked_trigger = None;
        }}
    }

    /// Compute one time step of the algorithm
    /// 
    pub fn step(&mut self, sensor: &mut SensorFusion) {

        if !self.step_started && sensor.position.z > STEP_START_THRESHOLD {
            self.step_started = true;

            cfg_if!{ if #[cfg(feature = "debug")] {
                self.step_start = Some(sensor.prev_time);
                println!("step-start: {}", sensor.prev_time);
            }}
        }

        if self.step_started && !self.step_peaked {
            if sensor.position.z > self.peak_pos.z {
                self.peak_pos.replace(&sensor.position);
                self.peak_gate = self.peak_pos.z / 2.0;
                cfg_if!{ if #[cfg(feature = "debug")] {
                    self.step_peak = Some(sensor.prev_time);
                }}
            }
        
            if sensor.position.z < self.peak_gate {
                self.step_peaked = true;

                cfg_if!{ if #[cfg(feature = "debug")] {
                    self.step_peaked_trigger = Some(sensor.prev_time);
                    println!("step-peak: {}", sensor.prev_time);
                    println!("step-peak-gate: {}", self.peak_gate);
                }}
            }
        }

        if self.step_peaked {
            if sensor.velocity.magnitude() < 0.05 {
                sensor.position = Vector::zero();
                self.peak_pos = Vector::zero();
                self.peak_gate = PEAK_GATE_DEFAULT;
                self.step_started = false;
                self.step_peaked = false;
            }
        }
    }
}