use hal::i2c::{Error, Instance, I2C};
use hal::Delay;
use hal::clock::Clocks;
use hal::prelude::*;
use math::{map_range, Quaternion, Vector};

use crate::{registers::*, AccelScaleRange, ClockSource, DLPFMode, GyroScaleRange, I2cSlave, SensorData, MPU6050_DEVICE_ID, MPU6505_DEFAULT_I2C_ADDR};
use crate::utils::*;
use crate::dmp::*;

pub struct Mpu6050<'a, 'b, T: Instance>
{
    /// i2c channel that we actually use to communicate with the MPU6050 chip.
    pub i2c: I2C<'a, T>,

    /// i2c address that chip is located at.
    address: u8,

    accel_scale: AccelScaleRange,
    gyro_scale: GyroScaleRange,

    // Clocks source to use for delays.
    clocks: &'b Clocks<'b>,
}

impl<'a, 'b, T: Instance> Mpu6050<'a, 'b, T>
{
    /// Create a new MPU 6050 instance with the given I2C interface.
    /// 
    pub fn new(i2c: I2C<'a, T>, clocks: &'b Clocks<'b>) -> Self {
        Mpu6050 {
            i2c,
            address: MPU6505_DEFAULT_I2C_ADDR,
            accel_scale: AccelScaleRange::default(),
            gyro_scale: GyroScaleRange::default(),
            clocks,
        }
    }

    /// Resets the MPU6050 chip. (This is used for waking the device up from sleep?)
    /// 
    pub fn reset(&mut self) -> Result<(), Error> {
        let mut delay = Delay::new(&self.clocks);
        self.i2c.write(self.address, &[ PWR_MGMT_1, 0x00 ])?;
        delay.delay_ms(350u32);
        Ok(())
    }

    pub fn set_accel_scale(&mut self, scale: AccelScaleRange) -> Result<(), Error> {
        self.i2c.write(self.address, &[ ACCEL_CONFIG, scale.as_register() ])?;
        self.accel_scale = scale;
        Ok(())
    }

    pub fn get_accel_scale(&mut self) -> Result<AccelScaleRange, Error> {
        let mut buf = [ 0u8 ];
        self.i2c.write_read(self.address, &[ ACCEL_CONFIG ], &mut buf)?;
        self.accel_scale = AccelScaleRange::from_register(buf[0]);
        Ok(self.accel_scale)
    }

    pub fn set_gyro_scale(&mut self, scale: GyroScaleRange) -> Result<(), Error> {
        let register = GYRO_CONFIG;
        let val = self.get_register_value(register)? & 0b1110_0111 + scale.as_register();
        self.gyro_scale = scale;
        self.set_register_value(register, val)
    }

    pub fn get_gyro_scale(&mut self) -> Result<GyroScaleRange, Error> {
        let mut buf = [ 0u8 ];
        self.i2c.write_read(self.address, &[ GYRO_CONFIG ], &mut buf)?;
        self.gyro_scale = GyroScaleRange::from_register(buf[0]);
        Ok(self.gyro_scale)
    }

    /// Get the current accelerometer sensor values (in deg/s).
    /// 
    pub fn get_accel(&mut self) -> Result<Vector, Error> {
        let mut data = [ 0u8; 6 ];
        self.i2c.write_read(self.address, &[ ACCEL_XOUT_H ], &mut data)?;
        let mut accel = [0.0f32; 3];
        for i in 0..3 {
            accel[i] = reg_to_f32(data[i*2], data[i*2+1]) / self.accel_scale.as_scale_factor();
        }
        Ok(Vector::from(accel))
    }

    /// Get the current gyroscope sensor values (in deg/s).
    /// 
    pub fn get_gyro(&mut self) -> Result<Vector, Error> {
        let mut data = [ 0u8; 6 ];
        self.i2c.write_read(self.address, &[ GYRO_XOUT_H ], &mut data)?;
        let mut gyros = [0.0f32; 3];
        for i in 0..3 {
            gyros[i] = reg_to_f32(data[i*2], data[i*2+1]) / self.gyro_scale.as_scale_factor();
        }
        Ok(Vector::from(gyros))
    }

    /// Get temperature of the on chip temperature sensor, result is returned in degrees celsius.
    /// 
    pub fn get_temp(&mut self) -> Result<f32, Error> {
        let mut data = [ 0u8; 2 ];
        self.i2c.write_read(self.address, &[ TEMP_OUT_H ], &mut data)?;
        // Formula from page 10 of register map data sheet.
        Ok(reg_to_f32(data[0], data[1]) / 340.0 + 36.53)
    }

    /// Gets the current gyroscope, acceleration, and temperature all at once, note that this is
    /// more efficient than calling `get_accel` and `get_gyro` after one another because this 
    /// method retrieves all data in a single ic2 transaction.
    /// 
    pub fn get_data(&mut self) -> Result<SensorData, Error> {
        let mut data = [ 0u8; 6+2+6 ];
        self.i2c.write_read(self.address, &[ ACCEL_XOUT_H ], &mut data)?;

        // First 6 bytes are accelerometer registers.
        let mut accel = [0.0f32; 3];
        for i in 0..3 {
            let bits = (&data[i*2 ..= i*2+1]).try_into().unwrap();
            accel[i] = (i16::from_be_bytes(bits) as f32) / self.accel_scale.as_scale_factor();
        }

        // Next 2 bytes are built-in temperature sensors.
        let bits: [u8; 2] = (&data[6 ..= 7]).try_into().unwrap();
        let temp = (i16::from_be_bytes(bits) as f32) / 340.0 + 36.53;

        // Last 6 bytes are gyroscope registers.
        let mut gyros = [0.0f32; 3];
        for i in 0..3 {
            let bits = (&data[i*2+8 ..= i*2+9]).try_into().unwrap();
            gyros[i] = (i16::from_be_bytes(bits) as f32) / self.gyro_scale.as_scale_factor();
        }

        Ok(SensorData { 
            accel: Vector::from(accel),
            gyros: Vector::from(gyros),
            temp
        })
    }

    pub fn calibrate_gyro(&mut self, loops: u8) -> Result<(), Error> {
        let mut kP: f64 = 0.3;
        let mut kI: f64 = 90.0;
        let x: f64 = (100.0 - map_range(loops as f64, 1.0, 5.0, 20.0, 0.0)) * 0.01;
        kP *= x;
        kI *= x;
        self.pid(0x43, &mut kP, &mut kI, loops)?;
        Ok(())
    }

    pub fn calibrate_accel(&mut self, loops: u8) -> Result<(), Error> {
        let mut kP: f64 = 0.3;
        let mut kI: f64 = 20.0;
        let x: f64 = (100.0 - map_range(loops as f64, 1.0, 5.0, 20.0, 0.0)) * 0.01;
        kP *= x;
        kI *= x;
        self.pid(0x3B, &mut kP, &mut kI, loops)?;
        Ok(())
    }

    fn pid(&mut self, read_address: u8, kP: &mut f64, kI: &mut f64, loops: u8) -> Result<(), Error> {
        let save_address: u8 = if read_address == ACCEL_XOUT_H { 
            if self.get_device_id()? < 0x38 { XA_OFFS_H } else { 0x77 } 
        } else { 
            XG_OFFS_USRH 
        };

        let mut delay = Delay::new(&self.clocks);

        let mut Data: i16;
        let mut Reading: f64;
        let mut BitZero = [0i16; 3];
        let shift: u8 = if save_address == 0x77 { 3 } else { 2 };
        let mut Error: f64;
        let mut PTerm: f64;
        let mut ITerm = [0.0f64; 3];
        let mut eSample: i16;
        let mut eSum: u32;
        let mut gravity: u16 = 8192; // prevent uninitialized compiler warning
        if read_address == 0x3B {
            gravity = 16384 >> (self.get_accel_scale()? as usize);
        }
        log::debug!(">");
        for i in 0..3usize {
            Data = self.get_register_value_i16(save_address + ((i as u8) * shift))?;
            Reading = Data as f64;
            if save_address != 0x13 {
                // Capture Bit Zero to properly handle Accelerometer calibration
                BitZero[i] = Data & 1;
                ITerm[i] = (Reading as f64) * 8.0;
            } 
            else {
                ITerm[i] = Reading * 4.0;
            }
        }
        for L in 0..loops {
            eSample = 0;
            for mut c in 0..100u8 {
                eSum = 0;
                for i in 0..3usize {
                    Data = self.get_register_value_i16(read_address + ((i as u8) * 2))?;
                    Reading = Data as f64;
                    if (read_address == 0x3B) && (i == 2) {
                        // remove Gravity
                        Reading -= gravity as f64;
                    }
                    Error = -Reading;
                    eSum += math::abs(Reading) as u32;
                    PTerm = (*kP) * Error;
                    ITerm[i] += (Error * 0.001) * (*kI);				// Integral term 1000 Calculations a second = 0.001
                    if save_address != 0x13 {
                        // Compute PID Output
                        Data = libm::round((PTerm + ITerm[i]) / 8.0) as i16;
                        // Insert Bit0 Saved at beginning
                        Data = raw_u16_to_i16((raw_i16_to_u16(Data) & 0xFFFE) | raw_i16_to_u16(BitZero[i]));
                    } else {
                        // Compute PID Output
                        Data = libm::round((PTerm + ITerm[i] ) / 4.0) as i16;
                    }
                    self.set_register_value_i16(save_address + ((i as u8) * shift), Data)?;
                }
                if (c == 99) && eSum > 1000 {						// Error is still to great to continue 
                    c = 0;
                    log::debug!("*");
                }
                if ((eSum as f32) * (if read_address== 0x3B { 0.05 } else { 1.0 })) < 5.0 {
                    // Successfully found offsets prepare to  advance
                    eSample += 1;
                }
                if (eSum < 100) && (c > 10) && (eSample >= 10) {
                    // Advance to next Loop
                    break;
                }
                delay.delay_micros(10);
            }
            log::debug!(".");
            *kP *= 0.75;
            *kI *= 0.75;
            for i in 0..3usize {
                if save_address != 0x1 {
                    //Compute PID Output
                    Data = libm::round((ITerm[i] ) / 8.0) as i16;
                    // Insert Bit0 Saved at beginning
                    Data = raw_u16_to_i16((raw_i16_to_u16(Data) & 0xFFFE) | raw_i16_to_u16(BitZero[i]));
                } else {
                    Data = libm::round((ITerm[i]) / 4.0) as i16;
                }
                self.set_register_value_i16(save_address + ((i as u8) * shift), Data)?;
            }
        }
        self.reset_fifo()?;
        self.reset_dmp()?;

        Ok(())
    }

    pub fn get_active_offsets(&mut self) -> Result<(Vector, Vector), Error> {
        let accel_offset_register: u8 = if self.get_device_id()? < 0x38 { XA_OFFS_H } else { 0x77 };

        let accel_offsets: Vector;
        if accel_offset_register == 0x06 {
            accel_offsets = self.get_register_value_vector(accel_offset_register)?;
        }
        else {
            accel_offsets = Vector {
                x: self.get_register_value_i16(accel_offset_register)? as f32,
                y: self.get_register_value_i16(accel_offset_register + 3)? as f32,
                z: self.get_register_value_i16(accel_offset_register + 6)? as f32,
            };
        
        }
        let gyro_offsets = self.get_register_value_vector(XG_OFFS_USRH)?;

        Ok((accel_offsets, gyro_offsets))
    }

    /// Get the MPU hardware revision, practically this reads a magical undocumented byte in the
    /// MPU's memory whose location was found in the `i2cdevlib` C++ library. If you actually want
    /// to use this number for something useful you should probably do more research.
    /// 
    pub fn get_hardware_revision(&mut self) -> Result<u8, Error> {

        self.set_memory_bank(0x10, true, true)?;
        self.set_memory_start_address(0x06)?;
        let mut revision = [ 0u8 ];
        self.i2c.write_read(self.address, &[ DMP_MEM_R_W ], &mut revision)?;
        self.set_memory_bank(0, false, false)?;

        Ok(revision[0])
    }

    /// Initializes the DMP (Digital Motion Processor) so data can be read from the FIFO queue,
    /// this needs to be called each time the sensor boots up.
    ///
    /// Because the DMP part of the chip seems to be mostly undocumented this method is essentially
    /// based completely on the `MPU6050_6Axis_MotionApps20::dmpInitialize()` function in Jeff
    /// Rowberg's excellent `i2cdevlib` C++ library.
    /// 
    pub fn initialize_dmp(&mut self) -> Result<(), Error> {

        let mut delay = Delay::new(&self.clocks);

        // Get MPU hardware revision, this might be optional/useless since we really don't do 
        // anything with the version.
        let revision = self.get_hardware_revision()?;
        log::info!("Hardware revision: {}", revision);

        // Check if OTP bank is valid, once again this may be optional since we don't really do
        // anything with it.
        let otp_bank_valid = self.get_otp_bank_valid()?;
        log::info!("OTP bank valid: {}", otp_bank_valid);
        
        if cfg!(feature = "dmp612") {
            // self.set_register_bit(PWR_MGMT_1, 7, true)?;
            // delay.delay_micros(100);
            // self.set_register_value(USER_CTRL, 0b0000_0111)?;
            // delay.delay_micros(100);
    
            // self.set_clock_source(ClockSource::GyroX)?;
            // self.set_register_value(INT_ENABLE, 0x00)?;
            // self.set_register_value(FIFO_EN, 0x00)?;
            // self.set_register_value(ACCEL_CONFIG, 0x00)?;
            // self.set_register_value(INT_PIN_CFG, 0x80)?;
            // self.set_clock_source(ClockSource::GyroX)?;
            // self.set_register_value(SMPLRT_DIV, 1)?;
            // self.set_register_value(CONFIG, 1)?;


            // // Set up some weird slave address stuff, no clue why this is done, there is no actual
            // // slave device connected.
            // self.set_slave_address(I2cSlave::Slave0, 0x07F)?;
            // self.set_i2c_master_mode(false)?;
            // self.set_slave_address(I2cSlave::Slave0, 0x068)?;
            // self.reset_i2c_master()?;
    
            // Wait for the change to take effect or something.
            self.reset()?;
            delay.delay_ms(120u32);
    
            self.set_clock_source(ClockSource::GyroZ)?;
            
            log::info!("Enabling DMP and FIFO_OFLOW interrupts");
            self.set_register_value(INT_ENABLE, 0b0001_0010)?;
    
            self.set_sample_rate_divider(4)?;
            self.set_external_frame_sync(1)?;
    
            self.set_dlpf_mode(DLPFMode::Bw42Hz)?;
            self.set_gyro_scale(GyroScaleRange::D2000)?;

            // Load DMP code into memory banks.
            self.write_memory_block(&DMP_FIRMWARE, 0, 0)?;
    
            self.set_register_value_u16(DMP_CFG_1, 0x0400)?;
            self.set_gyro_scale(GyroScaleRange::D2000)?;
            self.set_register_value(USER_CTRL, 0xC0)?;
            self.set_register_value(INT_ENABLE, 0x02)?;
            self.set_register_bit(USER_CTRL, 2, true)?;

            // disable DMP for compatibility with the MPU6050 library
            self.set_dmp_enabled(false)?;
        }
        else {

            // Set up some weird slave address stuff, no clue why this is done, there is no actual
            // slave device connected.
            self.set_slave_address(I2cSlave::Slave0, 0x07F)?;
            self.set_i2c_master_mode(false)?;
            self.set_slave_address(I2cSlave::Slave0, 0x068)?;
            self.reset_i2c_master()?;
    
            // Wait for the change to take effect or something.
            delay.delay_ms(120u32);
    
            self.set_clock_source(ClockSource::GyroZ)?;
            
            log::info!("Enabling DMP and FIFO_OFLOW interrupts");
            self.set_register_value(INT_ENABLE, 0b0001_0010)?;
    
            self.set_sample_rate_divider(4)?;
            self.set_external_frame_sync(1)?;
    
            self.set_dlpf_mode(DLPFMode::Bw42Hz)?;
            self.set_gyro_scale(GyroScaleRange::D2000)?;
    
            // Load DMP code into memory banks.
            self.write_memory_block(&DMP_FIRMWARE, 0, 0)?;
    
            // Set the FIFO Rate Divisor int the DMP Firmware Memory
            let dmp_update: [u8; 2] = [0x00, 0x01]; 
            self.write_memory_block(&dmp_update, 0x02, 0x16)?;
    
            // Setup DMP config
            self.set_register_value(DMP_CFG_1, 0x03)?;
            self.set_register_value(DMP_CFG_2, 0x00)?;
    
            self.set_otp_bank_valid(false)?;
    
            // Set motion detection threshold.
            self.set_register_value(MOT_THR, 2)?; 
            self.set_register_value(MOT_DUR, 80)?;
            self.set_register_value(ZERO_MOT_THR, 156)?;
            self.set_register_value(ZERO_MOT_DUR, 00)?;
    
            self.reset()?;
    
            self.set_fifo_enabled(true)?;
            self.reset_dmp()?;
            self.set_dmp_enabled(false)?;
            self.reset_fifo()?;
    
            // Clear interrupt flags.
            self.get_register_value(INT_STATUS)?;
        }

        log::info!("Finished setting up DMP");

        return Ok(());
    }

    /// Check if the DMP (Digital Motion Processor) is enabled.
    /// 
    pub fn get_dmp_enabled(&mut self) -> Result<bool, Error> {
        Ok(self.get_register_value(USER_CTRL)? & 0b1000_0000 > 0)
    }

    /// Enable or disable the DMP (Digital Motion Processor).
    /// 
    pub fn set_dmp_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b0111_1111; 
        if enabled { state += 0b1000_0000; }
        self.set_register_value(register, state)
    }

    /// Checks if there is currently at least one DMP packet worth of bytes in the FIFO queue.
    /// 
    pub fn dmp_packet_available(&mut self) -> Result<bool, Error> {
        Ok(self.get_fifo_count()? >= DMP_PACKET_SIZE)
    }

    /// Gets a DMP packet from the FIFO buffer, if one is available. Otherwise None is returned
    /// 
    /// 
    pub fn get_dmp_packet(&mut self) -> Option<DMPPacket> {
        if self.dmp_packet_available().ok()? {
            let mut bs = [ 0u8 ; (DMP_PACKET_SIZE as usize) ];
            self.i2c.write_read(self.address, &[ FIFO_R_W ], &mut bs).ok()?;

            let accel: Vector;
            let gyro: Vector;
            let quaternion: Quaternion;

            // The FIFO packet structure is different depending on the DMP firmware version.
            if cfg!(feature = "dmp20") {
                accel = Vector::new(
                    i32::from_be_bytes([ bs[28], bs[29], bs[30], bs[31] ]) as f32,  // X
                    i32::from_be_bytes([ bs[32], bs[33], bs[34], bs[35] ]) as f32,  // Y
                    i32::from_be_bytes([ bs[36], bs[37], bs[38], bs[39] ]) as f32,  // Z
                ) / (32768.0 * self.accel_scale.as_scale_factor());
            
                gyro = Vector::new(
                    i32::from_be_bytes([ bs[16], bs[17], bs[18], bs[19] ]) as f32,  // X
                    i32::from_be_bytes([ bs[20], bs[21], bs[22], bs[23] ]) as f32,  // Y
                    i32::from_be_bytes([ bs[24], bs[25], bs[26], bs[27] ]) as f32,  // Z
                ) / (self.gyro_scale.as_scale_factor() * 2048.0);
    
                quaternion = Quaternion::new(
                    (i32::from_be_bytes([ bs[0], bs[1], bs[2], bs[3] ]) as f32) / 16384.0,      // W
                    (i32::from_be_bytes([ bs[4], bs[5], bs[6], bs[7] ]) as f32) / 16384.0,      // X
                    (i32::from_be_bytes([ bs[8], bs[9], bs[10], bs[11] ]) as f32) / 16384.0,    // Y
                    (i32::from_be_bytes([ bs[11], bs[13], bs[14], bs[15] ]) as f32) / 16384.0,  // Z
                );
            }

            else if cfg!(feature = "dmp612") {
                accel = Vector::new(
                    i16::from_be_bytes([ bs[16], bs[17] ]) as f32,  // X
                    i16::from_be_bytes([ bs[18], bs[19] ]) as f32,  // Y
                    i16::from_be_bytes([ bs[20], bs[21] ]) as f32,  // Z
                ) / (32768.0 * self.accel_scale.as_scale_factor());
            
                gyro = Vector::new(
                    i16::from_be_bytes([ bs[22], bs[23] ]) as f32,  // X
                    i16::from_be_bytes([ bs[24], bs[25] ]) as f32,  // Y
                    i16::from_be_bytes([ bs[26], bs[27] ]) as f32,  // Z
                ) / (self.gyro_scale.as_scale_factor() * 2048.0);
    
                quaternion = Quaternion::new(
                    (i32::from_be_bytes([ bs[0], bs[1], bs[2], bs[3] ]) as f32) / 16384.0,      // W
                    (i32::from_be_bytes([ bs[4], bs[5], bs[6], bs[7] ]) as f32) / 16384.0,      // X
                    (i32::from_be_bytes([ bs[8], bs[9], bs[10], bs[11] ]) as f32) / 16384.0,    // Y
                    (i32::from_be_bytes([ bs[11], bs[13], bs[14], bs[15] ]) as f32) / 16384.0,  // Z
                );
            }

            else {
                panic!("No DMP firmware version configured!");
            }

            Some(DMPPacket {
                gyro, accel, quaternion
            })
        } 
        else { None }
    }

    /// Gets the number of bytes currently available inside FIFO buffer, generally speaking end
    /// users of this crate should not need this method. Use the `dmp_packet_available()` method
    /// instead.
    /// 
    pub fn get_fifo_count(&mut self) -> Result<u16, Error> {
        let mut data = [ 0u8; 2 ];
        self.i2c.write_read(self.address, &[ FIFO_COUNT_H ], &mut data)?;
        Ok(u16::from_be_bytes(data))
    }

    pub fn get_fifo_enabled(&mut self) -> Result<bool, Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b0100_0000; 
        Ok(state > 0)
    }

    pub fn set_fifo_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b1011_1111; 
        if enabled { state += 0b0100_0000; }
        self.set_register_value(register, state)
    }

    pub fn reset_dmp(&mut self) -> Result<(), Error> {
        log::debug!("Reset DMP");
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b1111_1011; 
        state += 0b0000_0100; 
        self.set_register_value(register, state)
    }

    /// Gets the device ID of this MPU6050 chip, practically speaking this just gets the contents
    /// of the `WHO_AM_I` register.
    /// 
    pub fn get_device_id(&mut self) -> Result<u8, Error> {
        let value = self.get_register_value(WHO_AM_I)?;
        Ok((value & 0b0111_1110) >> 1)
    }

    /// Checks if the i2c connection with the MPU6050 chip is working as expected, practically
    /// speaking this function just checks if it can read the device ID and if the device ID is
    /// the expected value.
    /// 
    pub fn connection_okay(&mut self) -> bool {
        self.get_device_id().map(|id| id == MPU6050_DEVICE_ID).unwrap_or(false)
    }

    /// This method resets the FIFO packet buffer, if the FIFO is NOT enabled.
    /// 
    pub fn reset_fifo(&mut self) -> Result<(), Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b1111_0111; 
        self.set_register_value(register, state)
    }

    /// Set digital low-pass filter configuration
    /// 
    pub fn set_dlpf_mode(&mut self, mode: DLPFMode) -> Result<(), Error> {
        let register = CONFIG;
        let mut state = self.get_register_value(register)?;
        state = (state & 0b1111_1000) + (mode as u8);
        self.set_register_value(register, state)
    }

    /// Sets the sample rate based on the divider using the following formula:
    /// `1khz / (1 + divider) = sample_rate`
    /// 
    /// For example: `1khz / (1 + 4) = 200 Hz`
    /// 
    pub fn set_sample_rate_divider(&mut self, divider: u8) -> Result<(), Error> {
        self.set_register_value(SMPLRT_DIV, divider)
    }

    /// Sets the i2c address at which the MPU6050 should expect to find the given slave.
    /// 
    pub fn set_slave_address(&mut self, slave: I2cSlave, address: u8) -> Result<(), Error> {
        self.i2c.write(self.address, &[ I2C_SLV0_ADDR + (slave as u8)*3, address ])
    }

    pub fn set_external_frame_sync(&mut self, sync: u8) -> Result<(), Error> {
        let sync = sync & 0b111;
        let register = CONFIG;
        let mut state = self.get_register_value(register)?;
        state &= 0b1100_0111;
        state += sync << 3;
        self.set_register_value(register, state)

    }

    pub fn set_clock_source(&mut self, source: ClockSource) -> Result<(), Error> {
        log::info!("Setting clock source={:?}", source);
        let register = PWR_MGMT_1;
        let mut state = self.get_register_value(register)?;
        state &= 0b1111_1000;
        state += source as u8;
        self.set_register_value(register, state)
    }

    pub fn set_i2c_master_mode(&mut self, enable: bool) -> Result<(), Error> {
        log::info!("Setting I2C master mode enabled={}", enable);
        // Get current register state
        let mut state = self.get_register_value(USER_CTRL)?;
        state &= 0b1101_1111;
        if enable { state += 0b0010_0000; }
        self.set_register_value(USER_CTRL, state)
    }

    pub fn reset_i2c_master(&mut self) -> Result<(), Error> {
        log::info!("Resetting I2C master mode");
        // Get current register state
        let mut state = self.get_register_value(USER_CTRL)?;
        state &= 0b1111_1101;
        state += 0b0000_0010;
        self.set_register_value(USER_CTRL, state)
    }

    pub fn get_register_value(&mut self, register: u8) -> Result<u8, Error> {
        let mut state = [ 0u8 ];
        self.i2c.write_read(self.address, &[ register ], &mut state)?;
        Ok(state[0])
    }

    pub fn set_register_value(&mut self, register: u8, value: u8) -> Result<(), Error> {
        self.i2c.write(self.address, &[ register, value ])
    }

    pub fn set_register_bit(&mut self, register: u8, bit: u8, enabled: bool) -> Result<(), Error> {
        if bit > 7 { 
            log::warn!("Skipping set bit because provided value was greater than 7.");
            return Ok(()); 
        }
        let mut val = self.get_register_value(register)?;
        let mask = 0b1111_1111 ^ (0b01 << bit);
        val = val & mask;
        if enabled {
            val += 0b01 << bit;
        }
        self.set_register_value(register, val)
    }

    pub fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<bool, Error> {
        let mut state = [ 0u8 ];
        self.i2c.write_read(self.address, &[ register ], &mut state)?;
        Ok(((state[0] >> bit) & 0b01) > 0)
    }

    /// Reads a signed 16 bit integer from the register and the next register, i.e. to read the
    /// ACCEL_XOUT_H and ACCEL_XOUT_L registers (at 0x03B and 0x03C respectively), you should call
    /// this method with ACCEL_XOUT_H as argument:
    /// 
    /// ```rs
    /// let accel_x = self.get_register_value_i16(ACCEL_XOUT_H)?;
    /// ```
    /// 
    pub fn get_register_value_i16(&mut self, register: u8) -> Result<i16, Error> {
        let mut state = [ 0u8, 0u8 ];
        self.i2c.write_read(self.address, &[ register ], &mut state)?;
        Ok(i16::from_be_bytes(state))
    }

    pub fn set_register_value_i16(&mut self, register: u8, value: i16) -> Result<(), Error> {
        let value = value.to_be_bytes();
        self.i2c.write(self.address, &[ register, value[0], value[1] ])
    }

    pub fn get_register_value_u16(&mut self, register: u8) -> Result<u16, Error> {
        let mut state = [ 0u8, 0u8 ];
        self.i2c.write_read(self.address, &[ register ], &mut state)?;
        Ok(u16::from_be_bytes(state))
    }

    pub fn set_register_value_u16(&mut self, register: u8, value: u16) -> Result<(), Error> {
        let value = value.to_be_bytes();
        self.i2c.write(self.address, &[ register, value[0], value[1] ])
    }

    /// Reads [u8; 6] registers interpreted as an [i16; 3] array and then converted to a f32 
    /// vector.
    /// 
    pub fn get_register_value_vector(&mut self, register: u8) -> Result<Vector, Error> {
        let mut state = [ 0u8; 6 ];
        self.i2c.write_read(self.address, &[ register ], &mut state)?;
        Ok(Vector {
            x: i16::from_be_bytes([state[0], state[1]]) as f32,
            y: i16::from_be_bytes([state[2], state[3]]) as f32,
            z: i16::from_be_bytes([state[4], state[5]]) as f32,
        })
    }

    fn get_otp_bank_valid(&mut self) -> Result<bool, Error> {
        let mut flags = [ 0u8 ];
        self.i2c.write_read(self.address, &[ XG_OFFS_TC ], &mut flags)?;
        Ok((flags[0] & 0b01) == 1)
    }
    
    fn set_otp_bank_valid(&mut self, valid: bool) -> Result<(), Error> {
        let register = XG_OFFS_TC;
        let mut state = self.get_register_value(register)?;
        state &= 0b1111_1110; 
        if valid { state += 1; }
        self.set_register_value(register, state)
    }
    
    fn write_memory_block(&mut self, data: &[u8], bank: u8, address: u8) -> Result<(), Error> {
        self.set_memory_bank(bank, false, false)?;
        self.set_memory_start_address(address)?;

        let data_size = data.len();
        let mut written: usize = 0;
        let mut _address = address as u16;
        let mut _bank = bank;
        while written < data_size 
        {
            let remaining = data_size - written;
            let left_in_bank = (DMP_MEMORY_BANK_SIZE - _address) as usize;
            let chunk_size = usize::min(DMP_MEMORY_CHUNK_SIZE, usize::min( remaining, left_in_bank));
            
            // log::info!("address {}, bank {}, written {}, remaining {}, left_in_bank {}, chunk_size {}", _address, _bank, written, remaining, left_in_bank, chunk_size);

            let mut chunk = [0u8; DMP_MEMORY_CHUNK_SIZE+1];
            chunk[0] = DMP_MEM_R_W;
            for i in 0..chunk_size {
                chunk[i+1] = data[written+i];
            }

            let chunk = &chunk[0..chunk_size+1];
            self.i2c.write(self.address, chunk)?;
            
            written += chunk_size;
            _address += chunk_size as u16;

            if _address >= u8::MAX as u16 {
                log::debug!("Finished writing to bank: {}", _bank);
                _address = 0;
                _bank += 1;
                self.set_memory_bank(_bank, false, false)?;
            }
            self.set_memory_start_address(_address as u8)?;
        }
        log::debug!("Finished writing to bank: {}", _bank);

        // for i in 0..data_size {
        //     let _bank = (i / (DMP_MEMORY_BANK_SIZE as usize)) as u8 + bank;
        //     let _address = ((i + address as usize) % (DMP_MEMORY_BANK_SIZE as usize)) as u8;
        //     self.set_memory_bank(_bank, false, false)?;
        //     self.set_memory_start_address(_address)?;
        //     self.i2c.write(self.address, &[DMP_MEM_R_W, data[i]])?;

        //     if i < 4 {
        //         log::info!("i: {}, bank: {}, address: {}, b: {}", i, _bank, _address, data[i]);
        //     }
        // }

        if cfg!(feature = "verify-firmware") 
        {
            let mut verify_data = [ 0u8 ];
            for i in 0..data_size {
                let _bank = (i / (DMP_MEMORY_BANK_SIZE as usize)) as u8 + bank;
                let _address = ((i + address as usize) % (DMP_MEMORY_BANK_SIZE as usize)) as u8;
                self.set_memory_bank(_bank, false, false)?;
                self.set_memory_start_address(_address)?;
                self.i2c.write_read(self.address, &[ DMP_MEM_R_W ], &mut verify_data)?;
                if verify_data[0] != data[i] {
                    if i < 4 {
                        log::error!(
                            "Verify of mem data failed: bank {}, address {}: found {} expected {}", 
                            _bank, _address, verify_data[0], data[i]
                        );
                    } else {
                        return Err(Error::TimeOut);
                    }
                }
            }
            log::debug!("Verified written data ok");
        }

        Ok(())
    }

    fn set_memory_start_address(&mut self, address: u8) -> Result<(), Error> {
        self.i2c.write(self.address, &[ DMP_MEM_START_ADDR, address ])
    }

    fn set_memory_bank(&mut self, mut bank: u8, prefetch: bool, user_bank: bool) -> Result<(), Error> {
        bank &= 0x1F;
        if user_bank {
            bank |= 0x20;
        }
        if prefetch {
            bank |= 0x40;
        }
        self.i2c.write(self.address, &[ DMP_BANK_SEL, bank ])
    }

}
