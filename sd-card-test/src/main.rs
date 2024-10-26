#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::Io, prelude::*, spi::{master::Spi, SpiMode}};
use embedded_sdmmc::SdCard;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let cs = io.pins.gpio6;
    let sck = io.pins.gpio8;
    let miso = io.pins.gpio9;
    let mosi = io.pins.gpio10;
    
    let mut spi = Spi::new(peripherals.SPI2, 20_000u32.kHz(), SpiMode::Mode0);
    spi = spi.with_pins(sck, mosi, miso, cs);

    let delay = Delay::new();
    let sdcard = SdCard::new(&mut spi, delay);

    loop {}
}
