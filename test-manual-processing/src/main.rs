#![no_std]
#![no_main]

use core::{cell::RefCell, panic::PanicInfo, sync::atomic::{AtomicBool, Ordering}};
use critical_section::Mutex;
use esp_println::{print, println};
use hal::{gpio::{Event, Input, Io, Pull}, i2c::master::I2c, prelude::*, rtc_cntl::Rtc};
use math::{EulerAngles, Vector, RAD_TO_DEG};
use mpu6050::{data, registers::INT_ENABLE, AccelScaleRange, ClockSource, DLPFMode, GyroScaleRange, Mpu6050, SensorData};
use processing::SensorFusion;
use hal::i2c::master::Config as I2cConfig;
use cfg_if::cfg_if;

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
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(on_sensor_ready);
    let rtc = Rtc::new(peripherals.LPWR);

    // Initialize I2C connection for the MPU6050
    // ============================================================================================
    let mut i2c = I2c::new(
        peripherals.I2C0, 
        I2cConfig {
            frequency: 400.kHz(),
            ..Default::default()
        }
    );
    cfg_if! { 
        if #[cfg(feature = "old-pins")] {
            i2c = i2c.with_sda(peripherals.GPIO1);
            i2c = i2c.with_scl(peripherals.GPIO2);
        } 
        else {
            i2c = i2c.with_sda(peripherals.GPIO8);
            i2c = i2c.with_scl(peripherals.GPIO7);
        }
    }


    // Setup an interrupt handler for the MPU6050 data ready interrupt pin
    // ============================================================================================
    let mut data_ready_pin;
    cfg_if! { if #[cfg(feature = "old-pins")] {
        data_ready_pin = Input::new(peripherals.GPIO0, Pull::Up)
    } else {
        data_ready_pin = Input::new(peripherals.GPIO21, Pull::Up);
    }};
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
    // We can ignore the first two bytes of the MAC address since these are always Expressive's
    // MAC prefix
    let mac = u32::from_be_bytes(hal::efuse::Efuse::read_base_mac_address()[2..6].try_into().unwrap());
    match mac {
        3673748608 => {
            mpu.set_active_offsets(
                &Vector::new(2416.0, 1621.0, 820.0),
                &Vector::new(3.0, 60.0, 36.0),
            ).unwrap();
        },
        3673747324 => {
            mpu.set_active_offsets(
                &Vector::new(52.0, -417.0, 1306.0),
                &Vector::new(138.0, 144.0, -15.0),
            ).unwrap();
        },
        3673708776 => {
            mpu.set_active_offsets(
                &Vector::new(-2264.0, -1729.0, 1458.0),
                &Vector::new(75.0, -29.0, -35.0),
            ).unwrap();
        },
        _ => {
            let mac = u32::from_be_bytes(hal::efuse::Efuse::read_base_mac_address()[2..6].try_into().unwrap());
            log::info!("mac_address: {:?}", mac);
        }
    }

    if cfg!(feature = "calibrate") {
        log::info!("Calibrating...");
        mpu.calibrate_accel(150).unwrap();
        mpu.calibrate_gyro(150).unwrap();
        let (acc_offset, gyro_offset) = mpu.get_active_offsets().unwrap();
        log::info!("mac_address: {:?}", mac);
        log::info!("acc_offset: {:?}", acc_offset);
        log::info!("gyro_offset: {:?}", gyro_offset);
        let mac = u32::from_be_bytes(hal::efuse::Efuse::read_base_mac_address()[2..6].try_into().unwrap());
            log::info!("mac_address: {:?}", mac);
        loop {}
    }





    // Main program loop
    // ============================================================================================
    let mut last_second = rtc.current_time().and_utc().timestamp();
    let mut counter = 0;
    let mut sensor_fusion = SensorFusion::new();
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

            sensor_fusion.step(
                time,
                data.accel,
                data.gyro,
            );

            if cfg!(feature = "debug-position") {
                let pos = sensor_fusion.position * 100;
                print!("{},{},{},", pos.x, pos.y, pos.z);
            }
            else if cfg!(feature = "debug-velocity") {
                let v = sensor_fusion.velocity * 100;
                print!("{},{},{},", v.x, v.y, v.z);
            }
            else if cfg!(feature = "debug-acceleration") {
                let a = sensor_fusion.world_acceleration * 100;
                print!("{},{},{},", a.x, a.y, a.z);
            }
            else if cfg!(feature = "debug-orientation") {
                let angles = EulerAngles::from(sensor_fusion.orientation);
                print!("{},{},{},", angles.yaw * RAD_TO_DEG, angles.pitch * RAD_TO_DEG, angles.roll * RAD_TO_DEG);
            }
            else if cfg!(feature = "debug-gravity") {
                let gravity = data.accel;
                print!("{},{},{},", gravity.x, gravity.y, gravity.z);
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
