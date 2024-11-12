//! BLE Example
//!
//! - starts Bluetooth advertising
//! - offers one service with three characteristics (one is read/write, one is write only, one is read/write/notify)
//! - pressing the boot-button on a dev-board will send a notification if it is subscribed

//% FEATURES: esp-wifi esp-wifi/ble
//% CHIPS: esp32 esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use bleps::{
    ad_structure::{
        create_advertising_data,
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::{AttributeServer, NotificationData, WorkResult},
    gatt,
    Ble,
    HciConnector,
};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, gpio::{Input, Io, Pull}, peripherals::*, prelude::*, rng::Rng, rtc_cntl::Rtc, system::SystemControl, timer::timg::TimerGroup
};
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, initialize, EspWifiInitFor};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    let init = initialize(
        EspWifiInitFor::Ble,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            let button = Input::new(io.pins.gpio0, Pull::Down);
        } else {
            let button = Input::new(io.pins.gpio9, Pull::Down);
        }
    }

    let mut debounce_cnt = 500;

    let mut bluetooth = peripherals.BT;

    loop {
        let connector = BleConnector::new(&init, &mut bluetooth);
        let hci = HciConnector::new(connector, esp_wifi::current_millis);
        let mut ble = Ble::new(&hci);

        println!("{:?}", ble.init());
        println!("{:?}", ble.cmd_set_le_advertising_parameters());
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName("Gait analyzer"),
                ])
                .unwrap()
            )
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true));

        println!("started advertising");

        let mut calibrate_sensor = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            20
        };

        let mut get_analyzing = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            20
        };

        let mut get_sys_time = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            20
        };

        let mut get_detection_queue = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            20
        };

        let mut get_device_state = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            20
        };

        let mut get_blink_led = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            20
        };

        let mut set_blink_led = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut set_analyzing = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [
                characteristic {
                    name: "calibrate_sensor",
                    uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
                    read: calibrate_sensor,
                },
                characteristic {
                    name: "analyzing",
                    uuid: "269026e7-fa94-4d80-ad51-35b9f2cf1f16",
                    read: get_analyzing,
                    write: set_analyzing,
                },
                characteristic {
                    name: "sys_time",
                    uuid: "4a483a4a-86f4-415c-9a3e-8e4b008af530",
                    read: get_sys_time,
                },
                characteristic {
                    name: "detection_queue",
                    uuid: "c359bc1e-b44e-4400-931c-08e1b89cb541",
                    read: get_detection_queue,
                },
                characteristic {
                    name: "device_state",
                    uuid: "7975e99d-0180-4ff7-890b-dc6aa558a08a",
                    read: get_device_state,
                },
                characteristic {
                    name: "blink_led",
                    uuid: "391c5495-c3f6-4e47-9baf-90dddbd3d525",
                    read: get_blink_led,
                    write: set_blink_led,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        loop {
            match srv.do_work() {
                Ok(res) => {
                    if let WorkResult::GotDisconnected = res {
                        println!("GotDisconnected");
                        break;
                    }
                }
                Err(err) => {
                    println!("{:?}", err);
                }
            }
        }
    }
}
