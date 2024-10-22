#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, entry, gpio::Io, i2c::{self, I2c}, peripherals::I2C0, prelude::*, time};

mod mpu5060;
use mpu5060::*;

fn run_program<'d>(mpu: &mut Mpu6050<'d, I2C0>) -> Result<(), i2c::Error> {
    loop {
        mpu.reset()?;
        mpu.set_clock_source(ClockSource::GyroX)?;
        mpu.set_accel_scale(AccelScaleRange::G4)?;
        mpu.set_gyro_scale(GyroScaleRange::D500)?;

        mpu.initialize_dmp()?;
        mpu.reset_fifo()?;

        let delay = Delay::new();
        delay.delay_millis(500);

        log::debug!("Connection okay: {:?}", mpu.connection_okay());
        log::debug!("DMP enabled: {:?}", mpu.get_dmp_enabled()?);
        log::debug!("FIFO enabled: {:?}", mpu.get_fifo_enabled()?);
        log::debug!("FIFO queue: {:?}", mpu.get_fifo_count()?);

        mpu.set_dmp_enabled(true)?;

        loop {
            // log::debug!("FIFO queue: {:?}", mpu5060.get_fifo_count());
            mpu.get_dmp_packet();
        }

    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let peripherals = esp_hal::init(esp_hal::Config::default());
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2c::new(
        peripherals.I2C0, 
        io.pins.gpio1,
        io.pins.gpio2,
        400.kHz()
    );

    let mut mpu6050 = Mpu6050::from(i2c);

    loop {
        if let Err(err) = run_program(&mut mpu6050) {
            log::error!("error: {:?}", err);
        }
    }
    
    // loop {
    //     let data = mpu6050.get_data().unwrap();
    //     let gyro = data.gyros;
    //     let temp = data.temp;
    //     let accel = data.accel;
    //     let now = time::now().duration_since_epoch().to_nanos();
    //     println!("{},{},{},{},{},{},{},{}", now, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, temp);
    // }
}
