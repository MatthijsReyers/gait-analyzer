#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write, panic::PanicInfo, sync::atomic::{AtomicBool, Ordering}};
use critical_section::Mutex;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, SdCardError, VolumeManager};
use hal::{delay::Delay, gpio::{Event, Input, Io, Output, Pull}, i2c::master::I2c, prelude::*, rtc_cntl::Rtc, spi::{master::Spi, SpiMode}};
use math::Vector;
use mpu6050::{registers::INT_ENABLE, AccelScaleRange, ClockSource, DLPFMode, GyroScaleRange, Mpu6050, SensorData};
use processing::SensorFusion;
use hal::spi::master::Config as SpiConfig;
use hal::i2c::master::Config as I2cConfig;
use cfg_if::cfg_if;

mod rtc_source;
use rtc_source::*;

mod bytewriter;
use bytewriter::*;

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

    
    // Initialize SPI interface for SD card.
    // ============================================================================================
    let mut spi = Spi::new_with_config(
        peripherals.SPI2,
        SpiConfig {
            frequency: 80.MHz(), 
            mode: SpiMode::Mode0,
            ..Default::default()
        } 
    );
    cfg_if! { if #[cfg(feature = "old-pins")] {
        spi = spi.with_mosi(peripherals.GPIO10);
        spi = spi.with_sck(peripherals.GPIO8);
        spi = spi.with_miso(peripherals.GPIO9);
    } else {
        spi = spi.with_sck(peripherals.GPIO3);
        spi = spi.with_miso(peripherals.GPIO9);
        spi = spi.with_mosi(peripherals.GPIO10);
    }}

    
    

    // Actually initialize SD card.
    // ============================================================================================
    let cs;
    cfg_if! { if #[cfg(feature = "old-pins")] {
        cs = Output::new(peripherals.GPIO6, hal::gpio::Level::High);
    } else {
        cs = Output::new(peripherals.GPIO4, hal::gpio::Level::High);
    }}
    let spi_device = ExclusiveDevice::new(&mut spi, cs, Delay::new()).unwrap();
    let sdcard = SdCard::new(spi_device, Delay::new());
    log::debug!("Checking for SD card.");
    match sdcard.num_bytes() {
        Ok(size) => log::info!("Found SD card with {} bytes.", size),
        Err(err) => {
            match err {
                SdCardError::CardNotFound => {
                    log::error!("No SD card found!");
                }
                _ => {
                    log::error!("Unknown SD card error!");
                }
            }
        }
    }


    // Initialize I2C connection for the MPU6050
    // ============================================================================================
    let mut i2c = I2c::new(
        peripherals.I2C0, 
        I2cConfig {
            frequency: 1100.kHz(),
            timeout: Some(120),
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
            log::warn!("Unknown ESP32, calibration required!");
        }
    }


    // Open the first volume in the partition table
    // ============================================================================================
    let mut volume_mgr = VolumeManager::new(sdcard, RtcSource {});
    let mut volume0 = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    let mut root_dir = volume0.open_root_dir().unwrap();


    // Look for the next free file number
    // ============================================================================================
    let mut buf = [0u8; 16];
    let mut file_name: &str = "0.csv";
    let mut writer = ByteWriter::new(&mut buf);
    for i in 0..99 {
        // Construct file name string
        writer.reset();
        writer.write_fmt(format_args!("{}.csv", i)).unwrap();
        let bs = writer.get_msg();
        file_name = unsafe { core::str::from_utf8_unchecked(bs) };
        log::debug!("Checking: {}", file_name);

        // Check if such a file already exists.
        if let Err(embedded_sdmmc::Error::NotFound) = root_dir.open_file_in_dir(file_name, embedded_sdmmc::Mode::ReadOnly) {
            log::info!("Found free file name: {}", file_name);
            break;
        }
    }

    
    // Write main csv header.
    // ============================================================================================
    let mut file = root_dir.open_file_in_dir(file_name, embedded_sdmmc::Mode::ReadWriteCreate).unwrap();
    file.write(b"time,gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z\n").unwrap();
    file.close().unwrap();


    // Main program loop
    // ============================================================================================
    let mut sensor_fusion = SensorFusion::new();
    loop {

        if SENSOR_READY.load(Ordering::Relaxed) {
            let data: SensorData = mpu.get_data().unwrap();
            SENSOR_READY.store(false, Ordering::Relaxed);
            let time = rtc.current_time().and_utc().timestamp_nanos_opt().unwrap();

            // sensor_fusion.step(
            //     time,
            //     data.accel,
            //     data.gyro,
            // );
            let mut file = root_dir.open_file_in_dir(file_name, embedded_sdmmc::Mode::ReadWriteAppend).unwrap();
            let mut buf = [0u8; 512];
            let mut writer = ByteWriter::new(&mut buf);

            writer.reset();
            writer.write_fmt(format_args!(
                "{},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8}\n", 
                time,
                data.gyro.x, data.gyro.y, data.gyro.z,
                data.accel.x, data.accel.y, data.accel.z,
            )).unwrap();
            file.write(writer.get_msg()).unwrap();

            file.close().unwrap();
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("Panicked with: {}", info);
    loop {}
    hal::reset::software_reset();
}
