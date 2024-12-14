#![no_std]
#![no_main]

extern crate alloc;
use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*};
use core::{cell::RefCell, sync::atomic::AtomicBool};
use critical_section::Mutex;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};

pub mod led;
pub mod bluetooth;
pub mod error;

pub type Global<T> = Mutex<RefCell<Option<T>>>;

/// Is the device currently analyzing motion or not?
/// 
pub static ANALYZING: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    led::setup(peripherals.GPIO8, peripherals.TIMG1);
    
    let mut bluetooth = peripherals.BT;
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let wifi_controller = esp_wifi::init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    ).unwrap();

    log::debug!("Initializing BLE");

    loop {
        if let Err(err) = bluetooth::run_bluetooth(&wifi_controller, &mut bluetooth) {
            log::error!("Application ran into an error: {:?}", err);
            let delay = Delay::new();
            delay.delay_micros(2000);
            log::info!("Restarting Bluetooth..");
        }
    }
}

