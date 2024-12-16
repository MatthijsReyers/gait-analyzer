#![no_std]
#![no_main]

extern crate alloc;
use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::{Event, Input, Io, Pull}, i2c::master::I2c, prelude::*, rtc_cntl::Rtc};
use esp_hal::i2c::master::Config as I2cConfig;
use queue::Queue;
use step::Step;
use core::cell::RefCell;
use critical_section::Mutex;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use sensor::{on_sensor_ready, Sensor, SENSOR_READY_PIN};
use cfg_if::cfg_if;

pub mod led;
pub mod bluetooth;
pub mod error;
pub mod sensor;
pub mod step;

pub type Global<T> = Mutex<RefCell<Option<T>>>;

pub static STEPS_QUEUE: Global<Queue<Step, 5>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });
    let rtc = Rtc::new(peripherals.LPWR);


    // Initialize the global steps queue.
    // ========================================================================
    critical_section::with(|cs| {
        STEPS_QUEUE.borrow(cs).replace(Some(Queue::new()));
    });


    // Setup i2c interface for MPU 5060 sensor communication.
    // ========================================================================
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
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(on_sensor_ready);
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


    // Setup on-board LED for debugging purposes
    // ========================================================================
    cfg_if! { 
        if #[cfg(feature = "old-pins")] {
            led::setup(peripherals.GPIO8, peripherals.TIMG1);
        }
    }
    

    // Setup BLE communication
    // ========================================================================
    let mut bluetooth = peripherals.BT;
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let wifi_controller = esp_wifi::init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    ).unwrap();


    let mut sensor = Sensor::new(i2c, rtc);
    if let Err(err) = sensor.setup_mpu() {
        log::error!("Failed to setup MPU6050: {:?}", err);
    } else {
        log::info!("Finished setting up MPU6050");
    }

    loop {
        if let Err(e) = bluetooth::run_bluetooth(
            &wifi_controller, 
            &mut bluetooth, 
            &mut sensor,
        ) {
            log::error!("Application ran into an error: {:?}", e);
            let delay = Delay::new();
            delay.delay_micros(2000);
            log::info!("Restarting Bluetooth..");
            if let Err(err) = sensor.reset() {
                log::error!("Sensor failed to reset: {:?}", err);
            }
        }
    }
}

