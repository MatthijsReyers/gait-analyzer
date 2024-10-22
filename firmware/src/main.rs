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

    let mut received_on: u64 = 0;
    let mut send_on: u64 = 0;

    loop {
        let now = current_millis();

        if send_on == 0 || (now - send_on > 1000 && received_on == 0) {
            send_on = current_millis();
            let data = send_on.to_be_bytes();
            let status = esp_now
                .send(&BROADCAST_ADDRESS, &data)
                .unwrap()
                .wait();
        }

        if let Some(r) = esp_now.receive() {
            let received = current_millis();
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
            let received_data = r.get_data();
            let received_timestamp = u64::from_be_bytes(received_data.try_into().unwrap());
            if received_timestamp == send_on {
                received_on = received;
            }
            let status = esp_now
                .send(&r.info.src_address, received_data)
                .unwrap()
                .wait();
        }

        if send_on != 0 && received_on != 0 {
            println!("Delta: {} ms", received_on - send_on);
            received_on = 0;
            send_on = 0;
        }
    }

}
