#![no_std]
#![no_main]

use core::panic::PanicInfo;

use embedded_sdmmc::{SdCard, SdCardError, Volume, VolumeManager};
// use esp_backtrace as _;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, spi::{master::Spi, SpiMode}, Delay, IO};
use embedded_sdmmc::{TimeSource, Timestamp};

pub struct RtcSource {}
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
fn main() -> ! {
    #[allow(unused)]
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let cs = io.pins.gpio4.into_push_pull_output();
    let sck = io.pins.gpio3;
    let miso = io.pins.gpio9;
    let mosi = io.pins.gpio10;

    
    // Initialize SPI interface.
    // ============================================================================================
    let mut spi = Spi::new(peripherals.SPI2, 500u32.kHz(), SpiMode::Mode0, &clocks);
    spi = spi.with_sck(sck);
    spi = spi.with_miso(miso);
    spi = spi.with_mosi(mosi);
    

    // Actually initialize SD card.
    // ============================================================================================
    log::debug!("Wait a minute");
    let delay = Delay::new(&clocks);
    delay.delay_micros(500);
    let sdcard = SdCard::new(spi, cs, delay);

    log::debug!("Checking init");

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
    log::debug!("Looking for partition 0 on SD card.");
    let mut volume_mgr = VolumeManager::new(sdcard, RtcSource {});
    let volume0: Volume = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    log::info!("Volume 0: {:?}", volume0);
    let root_dir = volume_mgr.open_root_dir(volume0).unwrap();
    let file = "test.txt";
    log::info!("Looking for file : {}", file);
    let my_file = volume_mgr.open_file_in_dir(root_dir, file, embedded_sdmmc::Mode::ReadOnly).unwrap();
    while !volume_mgr.file_eof(my_file).unwrap() {
        let mut buffer = [0u8; 32];
        let num_read = volume_mgr.read(my_file, &mut buffer);
        log::info!("num_read: {:?}", num_read);
        let num_read = num_read.unwrap();
        for b in &buffer[0..num_read] {
            log::info!("{}", *b as char);
        }
    }

    log::info!("Finished!");
    loop { }
}

#[panic_handler]
fn panic_handler<'b>(panic: &PanicInfo<'b>) -> ! {
    log::error!("Paniced!");
    log::error!("{:?}", panic);

    loop {}
}