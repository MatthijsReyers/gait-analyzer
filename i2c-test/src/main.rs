#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*, system::SystemControl
};
use esp_println::println;

mod registers;
use registers::*;

/// Default i2c address of the MPU 6050 chip.
/// 
const DEFAULT_I2C_ADDR: u8 = 0x68;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    
    esp_println::logger::init_logger_from_env();

    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        400.kHz(),
        &clocks,
    );

    i2c.write(DEFAULT_I2C_ADDR, &[ PWR_MGMT_1, 0b0_0000_0000 ]).unwrap();

    let mut values = [ 0u8; 6 ];
    loop {
        for (i, reg) in (ACCEL_XOUT_H..=(ACCEL_ZOUT_L)).enumerate()
        {
            let register = [ (reg << 1) + 1 ];
            if i2c.write_read(DEFAULT_I2C_ADDR, &register, &mut values[i..=i]).is_err() {
                println!("read_failed");
            }
        }

        let mut accel = [0u16; 3];
        for i in 0..3 {
            accel[i] = u16::from_be_bytes((&values[i*2..=i*2+1]).try_into().unwrap());
        }


        for (i, reg) in (GYRO_XOUT_H..=(GYRO_ZOUT_L)).enumerate()
        {
            let register = [ (reg << 1) + 1 ];
            if i2c.write_read(DEFAULT_I2C_ADDR, &register, &mut values[i..=i]).is_err() {
                println!("read_failed");
            }
        }

        let mut gyro = [0u16; 3];
        for i in 0..3 {
            gyro[i] = u16::from_be_bytes((&values[i*2..=i*2+1]).try_into().unwrap());
        }
        println!("accel: {:?}\ngyro: {:?}", accel, gyro);
    }
}
