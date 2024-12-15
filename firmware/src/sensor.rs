use core::{borrow::{Borrow, BorrowMut}, cell::RefCell, sync::atomic::{AtomicBool, Ordering}};
use critical_section::Mutex;
use esp_hal::{efuse::Efuse, gpio::Input, i2c::master::{AnyI2c, Error as I2cError, I2c}, macros::handler, rtc_cntl::Rtc, Blocking};
use math::Vector;
use mpu6050::{registers::INT_ENABLE, AccelScaleRange, ClockSource, DLPFMode, GyroScaleRange, Mpu6050, SensorData};
use crate::{error::AppError, step::Step, Global, STEPS_QUEUE};
use processing::*;

/// Is the device currently analyzing motion or not?
/// 
pub static ANALYZING: AtomicBool = AtomicBool::new(false);

/// The pin the MPU6050 will pull up when it has a new data packet for us.
/// 
pub static SENSOR_READY_PIN: Global<Input> = Mutex::new(RefCell::new(None));

/// Does the gyroscope/accelerometer have some new data ready for us? This flag is set by the
/// interrupt handler so the main loop knows that it needs to do something again.
/// 
static SENSOR_READY: AtomicBool = AtomicBool::new(false);

/// Interrupt handler for sensor ready pin, we only set the SENSOR_READY flag here to avoid
/// blocking important work done in the main loop by not actually getting and processing the main
/// data here.
/// 
#[handler]
pub fn on_sensor_ready() {
    critical_section::with(|cs| {
        if let Some(pin) = SENSOR_READY_PIN.borrow_ref_mut(cs).as_mut() {
            if pin.is_interrupt_set() {
                pin.clear_interrupt();
                SENSOR_READY.store(true, Ordering::Relaxed);
            }
        }
    });
}

pub struct Sensor {
    pub mpu: Mpu6050<'static, AnyI2c>,

    pub sensor_fusion: SensorFusion,
    pub step_detection: StepDetection,

    pub rtc: Rtc<'static>,
}

impl Sensor {

    pub fn new(i2c: I2c<'static, Blocking>, rtc: Rtc<'static>) -> Self 
    {
        Sensor {
            mpu: Mpu6050::new(i2c),
            sensor_fusion: SensorFusion::new(),
            step_detection: StepDetection::new(),
            rtc,
        }
    }

    pub fn start_analyzing(&mut self) -> Result<(), I2cError> {
        self.mpu.set_sleep(false)?;
        ANALYZING.store(true, Ordering::Relaxed);
        Ok(())
    }

    pub fn stop_analyzing(&mut self) -> Result<(), I2cError> {
        self.mpu.set_sleep(true)?;
        ANALYZING.store(false, Ordering::Relaxed);
        SENSOR_READY.store(false, Ordering::Relaxed);
        Ok(())
    }

    pub fn setup_mpu(&mut self) -> Result<(), I2cError> {
        log::info!("Configuring MPU6050");
        self.mpu.reset()?;
        self.mpu.set_clock_source(ClockSource::GyroX)?;
        self.mpu.set_accel_scale(AccelScaleRange::G8)?;
        self.mpu.set_gyro_scale(GyroScaleRange::D1000)?;
        self.mpu.set_dlpf_mode(DLPFMode::Bw10Hz)?;
        self.mpu.set_sample_rate_divider(7)?;
        self.mpu.set_register_value(INT_ENABLE, 0x01)?;
        self.mpu.set_dmp_enabled(false)?;
        self.mpu.set_fifo_enabled(false)?;

        // We can ignore the first two bytes of the MAC address since these are always Expressive's
        // MAC prefix
        let mac = u32::from_be_bytes(Efuse::read_base_mac_address()[2..6].try_into().unwrap());
        match mac {
            3673748608 => {
                let accel = Vector::new(2416.0, 1621.0, 820.0);
                let gyro = Vector::new(3.0, 60.0, 36.0);
                self.mpu.set_active_offsets(&accel, &gyro)?;
            },
            3673747324 => {
                let accel = Vector::new(52.0, -417.0, 1306.0);
                let gyro = Vector::new(138.0, 144.0, -15.0);
                self.mpu.set_active_offsets(&accel, &gyro)?;
            },
            3673708776 => {
                let accel = Vector { x: -2254.0, y: -1735.0, z: 1432.0 };
                let gyro = Vector { x: 58.0, y: -26.0, z: -32.0 };
                self.mpu.set_active_offsets(&accel, &gyro)?;
            },
            _ => {
                log::warn!("Unknown ESP32, calibration required!");
            }
        }

        self.stop_analyzing()
    }

    pub fn reset(&mut self) {
        self.stop_analyzing().unwrap();
        self.mpu.reset().unwrap();
    }

    pub fn do_work(&mut self) -> Result<(), AppError> {
        if SENSOR_READY.load(Ordering::Relaxed) {
            let data: SensorData = self.mpu.get_data()?;
            SENSOR_READY.store(false, Ordering::Relaxed);
            let time = self.rtc.current_time().and_utc().timestamp_nanos_opt().unwrap();

            self.sensor_fusion.update(time, &data.accel, &data.gyro);
            self.step_detection.update(&mut self.sensor_fusion, &data.accel);

            if self.step_detection.step_is_done() {
                let step = Step {
                    start: (self.step_detection.start_time.unwrap() / 1000) as u32,
                    stop: (self.step_detection.stop_time.unwrap() / 1000) as u32,
                };
                critical_section::with(|cs| {
                    if let Some(queue) = STEPS_QUEUE.borrow(cs).borrow_mut().as_mut() {
                        queue.push(step);
                    }
                });
            }
        }
        Ok(())
    }
}

