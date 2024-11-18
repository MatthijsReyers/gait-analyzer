#![no_std]
#![no_main]

use core::panic::PanicInfo;
#[cfg(feature = "sdcard")]
use embedded_sdmmc::{SdCard, SdCardError, TimeSource, Timestamp, Volume, VolumeManager};
use esp_println::print;
use hal::{clock::{ClockControl, Clocks}, delay, i2c::I2C, peripherals::Peripherals, prelude::*, spi::{master::Spi, SpiMode}, Delay, Rtc, IO};
use mpu6050::{AccelScaleRange, ClockSource, GyroScaleRange, Mpu6050};
use math::Vector;
use mpu6050::dmp::DMPPacket;

mod bytewriter;
use bytewriter::*;

#[cfg(feature = "sdcard")]
pub struct RtcSource {}
#[cfg(feature = "sdcard")]
impl<'d, 'a> TimeSource for RtcSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }   
    }
}

#[entry]
#[cfg(feature = "sdcard")]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);


    // Setup RTC
    // ============================================================================================
    let rtc = Rtc::new(peripherals.LPWR);


    // Initialize SPI interface.
    // ============================================================================================
    let mut spi = Spi::new(peripherals.SPI2, 20_000u32.kHz(), SpiMode::Mode0, &clocks);
    spi = spi.with_sck(io.pins.gpio8);
    spi = spi.with_miso(io.pins.gpio9);
    spi = spi.with_mosi(io.pins.gpio10);
    let cs = io.pins.gpio6.into_push_pull_output();
    

    // Actually initialize SD card.
    // ============================================================================================
    let delay = Delay::new(&clocks);
    let sdcard = SdCard::new(spi, cs, delay);
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
    let i2c = I2C::new(
        peripherals.I2C0, 
        io.pins.gpio1,
        io.pins.gpio2,
        hal::prelude::_fugit_RateExtU32::kHz(400),
        &clocks
    );


    // Setup DMP for the MPU6050
    // ============================================================================================
    let mut mpu = Mpu6050::new(i2c, &clocks);
    mpu.reset().unwrap();
    log::info!("MPU id: {}", mpu.get_device_id().unwrap());
    mpu.set_clock_source(ClockSource::GyroX).unwrap();

    // DO NOT CHANGE: These values are hardcoded in the firmware.
    mpu.set_accel_scale(AccelScaleRange::G4).unwrap();
    mpu.set_gyro_scale(GyroScaleRange::D2000).unwrap();

    mpu.set_dmp_enabled(false).unwrap();
    mpu.set_fifo_enabled(false).unwrap();

    mpu.initialize_dmp().unwrap();
    mpu.reset_fifo().unwrap();


    // Calibrate accelerometer
    // ============================================================================================
    log::info!("Calibrating...");
    mpu.calibrate_accel(250).unwrap();
    mpu.calibrate_gyro(250).unwrap();
    let (acc_offset, gyro_offset) = mpu.get_active_offsets().unwrap();
    log::info!("acc_offset: {:?}", acc_offset);
    log::info!("gyro_offset: {:?}", gyro_offset);


    // Open the first volume in the partition table
    // ============================================================================================
    let mut volume_mgr = VolumeManager::new(sdcard, RtcSource {});
    let volume0: Volume = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    let root_dir = volume_mgr.open_root_dir(volume0).unwrap();


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
        if let Err(embedded_sdmmc::Error::FileNotFound) = volume_mgr.open_file_in_dir(root_dir, file_name, embedded_sdmmc::Mode::ReadOnly) {
            log::info!("Found free file name: {}", file_name);
            break;
        }
    }


    // Open the available file and write the header
    // ============================================================================================
    let file = volume_mgr.open_file_in_dir(root_dir, file_name, embedded_sdmmc::Mode::ReadWriteCreateOrAppend).unwrap();
    volume_mgr.write(file, b"time,").unwrap();
    volume_mgr.write(file, b"gyro.x,gyro.y,gyro.z,").unwrap();
    volume_mgr.write(file, b"accel.x,accel.y,accel.z,").unwrap();
    volume_mgr.write(file, b"dmp.gyro.x,dmp.gyro.y,dmp.gyro.z,").unwrap();
    volume_mgr.write(file, b"dmp.accel.x,dmp.accel.y,dmp.accel.z,").unwrap();
    volume_mgr.write(file, b"dmp.quat.w,dmp.quat.x,dmp.quat.y,dmp.quat.z\n").unwrap();
    volume_mgr.close_file(file).unwrap();
    log::info!("Wrote headers to file");


    // Main program loop
    // ============================================================================================
    mpu.set_fifo_enabled(true).unwrap();
    mpu.set_dmp_enabled(true).unwrap();
    let mut previous_t = rtc.get_time_us();
    loop {
        if let Some(packet) = mpu.get_dmp_packet() {
            let file = volume_mgr.open_file_in_dir(root_dir, file_name, embedded_sdmmc::Mode::ReadWriteAppend).unwrap();
            let time = rtc.get_time_us();
            let gyro = mpu.get_gyro().unwrap();
            let accel = mpu.get_accel().unwrap();

            let mut buf = [0u8; 256];
            let mut writer = ByteWriter::new(&mut buf);

            writer.reset();
            writer.write_fmt(format_args!("{},", time)).unwrap();
            volume_mgr.write(file, writer.get_msg()).unwrap();

            writer.reset();
            writer.write_fmt(format_args!("{},{},{},", gyro.x, gyro.y, gyro.z)).unwrap();
            volume_mgr.write(file, writer.get_msg()).unwrap();

            writer.reset();
            writer.write_fmt(format_args!("{},{},{},", accel.x, accel.y, accel.z)).unwrap();
            volume_mgr.write(file, writer.get_msg()).unwrap();

            writer.reset();
            writer.write_fmt(format_args!("{},{},{},", packet.gyro.x, packet.gyro.y, packet.gyro.z)).unwrap();
            volume_mgr.write(file, writer.get_msg()).unwrap();

            writer.reset();
            writer.write_fmt(format_args!("{},{},{},", packet.accel.x, packet.accel.y, packet.accel.z)).unwrap();
            volume_mgr.write(file, writer.get_msg()).unwrap();

            writer.reset();
            writer.write_fmt(format_args!(
                "{},{},{},{}\n", 
                packet.quaternion.w, 
                packet.quaternion.x, 
                packet.quaternion.y,
                packet.quaternion.z,
            )).unwrap();
            volume_mgr.write(file, writer.get_msg()).unwrap();

            log::info!("delta: {}", time - previous_t);
            previous_t = time;
            volume_mgr.close_file(file).unwrap();
        }
    }
}

#[entry]
#[cfg(not(feature = "sdcard"))]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Debug);
    let peripherals = hal::init(hal::Config::default());
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);


    // Setup RTC
    // ============================================================================================
    let rtc = Rtc::new(peripherals.LPWR);


    // Initialize I2C connection for the MPU6050
    // ============================================================================================
    let i2c = I2C::new(
        peripherals.I2C0, 
        io.pins.gpio1,
        io.pins.gpio2,
        hal::prelude::_fugit_RateExtU32::kHz(400),
        &clocks
    );


    // Setup DMP for the MPU6050
    // ============================================================================================
    let mut mpu = Mpu6050::new(i2c);
    mpu.reset().unwrap();
    log::info!("MPU id: {}", mpu.get_device_id().unwrap());
    mpu.set_clock_source(ClockSource::GyroX).unwrap();

    mpu.set_clock_source(ClockSource::GyroX).unwrap();
    mpu.set_accel_scale(AccelScaleRange::G2).unwrap();
    mpu.set_gyro_scale(GyroScaleRange::D2000).unwrap();
    mpu.set_sample_rate_divider(4).unwrap();
    mpu.set_register_value(INT_ENABLE, 0x01).unwrap();
    mpu.set_dmp_enabled(false).unwrap();
    mpu.set_fifo_enabled(false).unwrap();

    // // DO NOT CHANGE: These values are hardcoded in the firmware.
    // mpu.set_accel_scale(AccelScaleRange::G2).unwrap();
    // mpu.set_gyro_scale(GyroScaleRange::D2000).unwrap();

    // mpu.set_dmp_enabled(false).unwrap();
    // mpu.set_fifo_enabled(false).unwrap();

    // mpu.initialize_dmp().unwrap();
    // mpu.reset_fifo().unwrap();


    // Calibrate accelerometer
    // ============================================================================================
    log::info!("Calibrating...");
    mpu.calibrate_accel(50).unwrap();
    mpu.calibrate_gyro(50).unwrap();
    let (acc_offset, gyro_offset) = mpu.get_active_offsets().unwrap();
    log::info!("acc_offset: {:?}", acc_offset);
    log::info!("gyro_offset: {:?}", gyro_offset);


    // Main program loop
    // ============================================================================================
    mpu.set_fifo_enabled(true).unwrap();
    mpu.set_dmp_enabled(true).unwrap();
    let mut previous_t = rtc.get_time_us();
    let mut avg = Vector::new(0.0, 0.0, 0.0);
    loop {
        if let Some(packet) = mpu.get_dmp_packet() {
            let packet: DMPPacket = packet;
            let time = rtc.get_time_us();

            print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
            print!("x: {: >7.2} deg\n", packet.gyro.x);
            print!("y: {: >7.2} deg\n", packet.gyro.y);
            print!("z: {: >7.2} deg\n", packet.gyro.z);

            // print!("{},", time);
            // print!("{},{},{},", gyro.x, gyro.y, gyro.z);
            // print!("{},{},{},", accel.x, accel.y, accel.z);
            // print!("{},{},{},", packet.gyro.x, packet.gyro.y, packet.gyro.z);
            // print!("{},{},{},", packet.accel.x, packet.accel.y, packet.accel.z);
            // print!(
            //     "{},{},{},{}\n", 
            //     packet.quaternion.w, 
            //     packet.quaternion.x, 
            //     packet.quaternion.y,
            //     packet.quaternion.z,
            // );
            
            previous_t = time;

            let gravity = packet.get_gravity().normalize();
            let mut ypr = packet.get_yaw_pitch_roll(&gravity);

            // Turn PI into degrees
            ypr *= 57.2958;

            avg = avg*0.9 + ypr*0.1;

            print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
            print!("yaw:   {: >7.2} deg\n", avg.x);
            print!("pitch: {: >7.2} deg\n", avg.y);
            print!("roll:  {: >7.2} deg\n", avg.z);
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
