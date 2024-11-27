use crate::*;

static STEP_START_THRESHOLD: f32 = 0.05;

pub struct StepDetection
{
    step_started: bool,
    max_offset: Vector,
    step_stopped: bool,
}

impl StepDetection
{
    #[inline]
    pub fn new() -> Self {
        StepDetection {
            step_started: false,
            
            max_offset: Vector::zero(),

            step_stopped: false,
        }
    }

    /// Compute one time step of the algorithm
    /// 
    pub fn step(&mut self, sensor: &mut SensorFusion) {

        if !self.step_started && sensor.position.z > STEP_START_THRESHOLD + 0.01 {
            self.step_started = true;

            if cfg!(feature = "csv") {
                println!("step-start: {}", sensor.prev_time);
            }
        }

        if self.step_started && !self.step_stopped {
            self.max_offset.x = libm::fmaxf(sensor.position.x, self.max_offset.x);
            self.max_offset.y = libm::fmaxf(sensor.position.y, self.max_offset.y);
            self.max_offset.z = libm::fmaxf(sensor.position.z, self.max_offset.z);
        
            if sensor.position.z < (self.max_offset.z / 2.0) {
                self.step_stopped = true;
            }
        }

        if self.step_stopped {
            if sensor.velocity.magnitude() < 0.1 {
                sensor.position = Vector::zero();
                self.max_offset = Vector::zero();
                self.step_started = false;
                self.step_stopped = false;
            }
        }
    }
}