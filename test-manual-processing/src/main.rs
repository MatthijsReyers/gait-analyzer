#![no_std]
#![no_main]

use core::{cell::RefCell, panic::PanicInfo, sync::atomic::{AtomicBool, Ordering}};
use critical_section::Mutex;
use esp_println::{print, println};
use hal::{gpio::{Event, Input, Io, Pull}, i2c::I2c, prelude::*, rtc_cntl::Rtc};
use math::{EulerAngles, RAD_TO_DEG};
use mpu6050::{registers::INT_ENABLE, AccelScaleRange, ClockSource, DLPFMode, GyroScaleRange, Mpu6050, SensorData};
use processing::ProcessingAlgorithm;

type Global<T> = Mutex<RefCell<Option<T>>>;

/// 
static SENSOR_READY_PIN: Global<Input> = Mutex::new(RefCell::new(None));

/// Has the gyroscope/accelerometer some new data ready for us?
static SENSOR_READY: AtomicBool = AtomicBool::new(false);

#[handler]
fn on_sensor_ready() {
    critical_section::with(|cs| {
        if let Some(pin) = SENSOR_READY_PIN.borrow_ref_mut(cs).as_mut() {
            if pin.is_interrupt_set() {
                pin.clear_interrupt();
                SENSOR_READY.store(true, Ordering::Relaxed);
            }
        }
    });
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let peripherals = hal::init(hal::Config::default());
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(on_sensor_ready);
    let rtc = Rtc::new(peripherals.LPWR);


    // Initialize I2C connection for the MPU6050
    // ============================================================================================
    let i2c = I2c::new(
        peripherals.I2C0, 
        io.pins.gpio1,
        io.pins.gpio2,
        400.kHz()
    );


    // Setup an interrupt handler for the MPU6050 data ready interrupt pin
    // ============================================================================================
    let mut data_ready_pin = Input::new(io.pins.gpio0, Pull::Up);
    critical_section::with(|cs| {
        data_ready_pin.listen(Event::RisingEdge);
        SENSOR_READY_PIN.borrow_ref_mut(cs).replace(data_ready_pin);
    });


    // Setup the MPU6050
    // ============================================================================================
    let mut mpu = Mpu6050::new(i2c);
    mpu.reset().unwrap();
    log::info!("MPU id: {}", mpu.get_device_id().unwrap());

    mpu.set_clock_source(ClockSource::GyroX).unwrap();
    mpu.set_accel_scale(AccelScaleRange::G2).unwrap();
    mpu.set_gyro_scale(GyroScaleRange::D250).unwrap();
    mpu.set_dlpf_mode(DLPFMode::Bw10Hz).unwrap();
    mpu.set_sample_rate_divider(4).unwrap();
    mpu.set_register_value(INT_ENABLE, 0x01).unwrap();
    mpu.set_dmp_enabled(false).unwrap();
    mpu.set_fifo_enabled(false).unwrap();


    // Calibrate accelerometer
    // ============================================================================================
    // log::info!("Calibrating...");
    // mpu.calibrate_accel(50).unwrap();
    // mpu.calibrate_gyro(50).unwrap();
    // let (acc_offset, gyro_offset) = mpu.get_active_offsets().unwrap();
    // log::info!("acc_offset: {:?}", acc_offset);
    // log::info!("gyro_offset: {:?}", gyro_offset);

    
    // Main program loop
    // ============================================================================================
    let mut last_second = rtc.current_time().and_utc().timestamp();
    let mut counter = 0;

    let mut algo = ProcessingAlgorithm::new();

    println!("time,gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z");

    loop {
        if SENSOR_READY.load(Ordering::Relaxed) {
            let data: SensorData = mpu.get_data().unwrap();
            
            SENSOR_READY.store(false, Ordering::Relaxed);
            
            counter += 1;

            let time = rtc.current_time().and_utc().timestamp_nanos_opt().unwrap();

            // Reset the packet counter every second.
            let current_second = rtc.current_time().and_utc().timestamp();
            if current_second > last_second {
                last_second = current_second;
                counter = 0;
            }
            
            algo.step(
                time,
                data.accel,
                data.gyro,
            );

            if cfg!(feature = "debug-position") {
                print!("{},{},{},", algo.position.x, algo.position.y, algo.position.z);
            }
            else if cfg!(feature = "debug-velocity") {
                print!("{},{},{},", algo.velocity.x, algo.velocity.y, algo.velocity.z);
            }
            else if cfg!(feature = "debug-acceleration") {
                print!("{},{},{},", algo.world_acceleration.x, algo.world_acceleration.y, algo.world_acceleration.z);
            }
            else if cfg!(feature = "debug-orientation") {
                let angles = EulerAngles::from(algo.orientation);
                print!("{},{},{},", angles.yaw * RAD_TO_DEG, angles.pitch * RAD_TO_DEG, angles.roll * RAD_TO_DEG);
            }
            else {
                print!(
                    "{},{:.32},{:.32},{:.32},{:.32},{:.32},{:.32}", 
                    time,
                    data.gyro.x,
                    data.gyro.y,
                    data.gyro.z,
                    data.accel.x,
                    data.accel.y,
                    data.accel.z,
                );
            }

            print!("\n");
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("Panicked with: {}", info);
    hal::reset::software_reset();
    loop {}
}
