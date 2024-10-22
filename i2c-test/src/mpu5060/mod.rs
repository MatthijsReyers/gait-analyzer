use esp_hal::{delay::Delay, i2c::{Error, I2c, Instance}, Blocking};

pub mod vector;
pub use vector::*;

pub mod accel_scale_range;
pub use accel_scale_range::*;

pub mod gyro_scale_range;
pub use gyro_scale_range::*;

pub mod data;
pub use data::*;

pub mod i2c_slave;
pub use i2c_slave::*;

pub mod clock_source;
pub use clock_source::*;

pub mod dlpf_mode;
pub use dlpf_mode::*;

pub mod registers;
use registers::*;

pub mod utils;
use utils::*;

pub mod dmp;
use dmp::*;

/// Default i2c address of the MPU 6050 chip.
/// 
pub const MPU6505_DEFAULT_I2C_ADDR: u8 = 0x68;

/// The default device ID of a MPU6050 chip.
/// 
pub const MPU6050_DEVICE_ID: u8 = 0x034;

pub struct Mpu6050<'d, T: Instance>
{
    /// i2c channel that we actually use to communicate with the MPU6050 chip.
    i2c: I2c<'d, T, Blocking>,

    /// i2c address that chip is located at.
    address: u8,

    accel_scale: AccelScaleRange,
    gyro_scale: GyroScaleRange,
}

impl<'d, T: Instance> Mpu6050<'d, T>
{
    /// Resets the MPU6050 chip. (This is used for waking the device up from sleep?)
    /// 
    pub fn reset(&mut self) -> Result<(), Error> {
        let delay = Delay::new();
        self.i2c.write(self.address, &[ PWR_MGMT_1, 0x00 ])?;
        delay.delay_millis(350);
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
        self.i2c.write(self.address, &[ GYRO_CONFIG, scale.as_register() ])?;
        self.gyro_scale = scale;
        Ok(())
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

    /// Initializes the DMP (Digital Motion Processor) so data can be read from the FIFO queue,
    /// this needs to be called each time the sensor boots up.
    ///
    /// Because the DMP part of the chip seems to be mostly undocumented this method is essentially
    /// based completely on the `MPU6050_6Axis_MotionApps20::dmpInitialize()` function in Jeff
    /// Rowberg's excellent `i2cdevlib` C++ library.
    /// 
    pub fn initialize_dmp(&mut self) -> Result<(), Error> {
        let delay = Delay::new();

        // Reset the device so it's not sleeping or anything.
        self.reset()?;
        delay.delay_millis(30);

        // Get MPU hardware revision, this might be optional/useless since we really don't do 
        // anything with the version.
        self.set_memory_bank(0x10, true, true)?;
        self.set_memory_start_address(0x60)?;
        let mut revision = [ 0x0FFu8 ];
        self.i2c.write_read(self.address, &[ DMP_MEM_R_W ], &mut revision)?;
        log::info!("Hardware revision: {}", revision[0]);
        self.set_memory_bank(0, false, false)?;

        // Check if OTP bank is valid, once again this may be optional since we don't really do
        // anything with it.
        let otp_bank_valid = self.get_otp_bank_valid()?;
        log::info!("OTP bank valid: {}", otp_bank_valid);

        // It sets up weird slave address stuff, no clue why
        self.set_slave_address(I2cSlave::Slave0, 0x07F)?;
        self.set_i2c_master_mode(false)?;
        self.set_slave_address(I2cSlave::Slave0, 0x068)?;
        self.reset_i2c_master()?;
        delay.delay_millis(20);

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
        let dmp_update: [u8; 2] = [0x00, 0x00]; 
        self.write_memory_block(&dmp_update, 0x02, 0x16)?;

        // Setup DMP config
        self.set_register_value(DMP_CFG_1, 0x03)?;
        self.set_register_value(DMP_CFG_1, 0x00)?;

        self.set_otp_bank_valid(false)?;

        // Set motion detection threshold.
        self.set_register_value(MOT_THR, 2)?; 
        self.set_register_value(MOT_DUR, 80)?;
        self.set_register_value(ZERO_MOT_THR, 156)?;
        self.set_register_value(ZERO_MOT_DUR, 00)?;

        self.set_fifo_enabled(true)?;
        self.reset_dmp()?;
        self.set_dmp_enabled(false)?;
        self.reset_fifo()?;

        // Clear interrupt flags.
        self.get_register_value(INT_STATUS)?;

        log::info!("Finished setting up DMP");

        return Ok(());
    }

    pub fn set_dmp_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b0111_1111; 
        if enabled { state += 0b1000_0000; }
        self.set_register_value(register, state)
    }

    pub fn get_dmp_enabled(&mut self) -> Result<bool, Error> {
        Ok(self.get_register_value(USER_CTRL)? & 0b1000_0000 > 0)
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
            let mut data = [ 0u8 ; (DMP_PACKET_SIZE as usize) ];
            self.i2c.write_read(self.address, &[ FIFO_R_W ], &mut data).ok()?;
            log::debug!("packet: {:?}", data);
            return Some(DMPPacket::from_bytes(data));
        }
        None
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

    fn write_memory_block(&mut self, data: &[u8], bank: u8, address: u8) -> Result<(), Error> 
    {
        self.set_memory_bank(bank, false, false)?;
        self.set_memory_start_address(address)?;

        let data_size = data.len();
        let mut written: usize = 0;
        let mut _address = address as u16;
        let mut _bank = bank;
        while written < data_size {
            
            let remaining = data_size - written;
            let left_in_bank = (DMP_MEMORY_BANK_SIZE - _address) as usize;
            let chunk_size = usize::min(DMP_MEMORY_CHUNK_SIZE, usize::min( remaining, left_in_bank));
            
            // log::info!("address {}, bank {}, written {}, remaining {}, left_in_bank {}, chunk_size {}", _address, bank, written, remaining, left_in_bank, chunk_size);

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

        // TODO: Read back the data to verify that it is actually written to the device RAM correctly.

        let mut verify_data = [ 0u8 ];
        for i in 0..data_size {
            let _bank = (i / (DMP_MEMORY_BANK_SIZE as usize)) as u8 + bank;
            let _address = ((i + address as usize) % (DMP_MEMORY_BANK_SIZE as usize)) as u8;
            self.set_memory_bank(_bank, false, false)?;
            self.set_memory_start_address(_address)?;
            self.i2c.write_read(self.address, &[ DMP_MEM_R_W ], &mut verify_data)?;
            if verify_data[0] != data[i] {
                log::error!(
                    "Verify of mem data failed: bank {}, address {}: found {} expected {}", 
                    _bank, _address, verify_data[0], data[i]
                );
                panic!("Verify of written data failed.")
            }
        }
        log::debug!("Verified written data ok");

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

    pub fn set_fifo_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b1011_1111; 
        if enabled { state += 0b0100_0000; }
        self.set_register_value(register, state)
    }

    pub fn get_fifo_enabled(&mut self) -> Result<bool, Error> {
        let register = USER_CTRL;
        let mut state = self.get_register_value(register)?;
        state &= 0b0100_0000; 
        Ok(state > 0)
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
}

impl<'d, T: Instance> From<I2c<'d, T, Blocking>> for Mpu6050<'d, T>
{
    fn from(i2c: I2c<'d, T, Blocking>) -> Self {
        Mpu6050 {
            i2c,
            address: MPU6505_DEFAULT_I2C_ADDR,
            accel_scale: AccelScaleRange::default(),
            gyro_scale: GyroScaleRange::default(),
        }
    }
}
