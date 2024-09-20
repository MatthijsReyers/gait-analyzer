#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks}, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*, rng::Rng, system::SystemControl, timer::timg::TimerGroup
};
use esp_println::println;
use esp_wifi::{current_millis, esp_now::{PeerInfo, BROADCAST_ADDRESS}, EspWifiInitFor};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
    );

    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    ).unwrap();

    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();

    println!("esp-now version {}", esp_now.get_version().unwrap());

    let mut next_send_time = current_millis() + 5 * 1000;
    loop {
        let r = esp_now.receive();
        if let Some(r) = r {
            println!("Received {:?}", r);

            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                let status = esp_now
                    .send(&r.info.src_address, b"Hello Peer")
                    .unwrap()
                    .wait();
                println!("Send hello to peer status: {:?}", status);
            }
        }

        if current_millis() >= next_send_time {
            next_send_time = current_millis() + 5 * 1000;
            println!("Send");
            let status = esp_now
                .send(&BROADCAST_ADDRESS, b"0123456789")
                .unwrap()
                .wait();
            println!("Send broadcast status: {:?}", status)
        }
    }
}
