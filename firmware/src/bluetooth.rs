use bleps::{ad_structure::{create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE}, attribute_server::{AttributeServer, WorkResult}, gatt, Ble, HciConnector};
use esp_hal::{peripheral::Peripheral, time};
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, EspWifiController};

use crate::{error::AppError, led};


pub fn run_bluetooth(
    wifi_controller: &EspWifiController, 
    bluetooth: &mut impl Peripheral<P = esp_hal::peripherals::BT>
) -> Result<(), AppError> {

    let connector = BleConnector::new(&wifi_controller, bluetooth);
    let now = || time::now().duration_since_epoch().to_millis();
    let hci = HciConnector::new(connector, now);
    let mut ble = Ble::new(&hci);

    log::debug!("Initializing BLE");
    ble.init()?;

    log::debug!("Setting LE advertising parameters");
    ble.cmd_set_le_advertising_parameters()?;
   
    log::debug!("Setting LE advertising data");
    ble.cmd_set_le_advertising_data(
        create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
            AdStructure::CompleteLocalName("Gait analyzer"),
        ])?
    )?;

    log::debug!("Enabling LE advertising");
    ble.cmd_set_le_advertise_enable(true)?;

    let mut calibrate_sensor = |_offset: usize, data: &mut [u8]| {
        data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
        20
    };

    let mut get_analyzing = |_offset: usize, data: &mut [u8]| {
        data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
        20
    };

    let mut set_analyzing = |offset: usize, data: &[u8]| {
        println!("RECEIVED: {} {:?}", offset, data);
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
        data[0] = if led::get_blinking() { 0xFF } else { 0x00 };
        1
    };

    let mut set_blink_led = |_offset: usize, data: &[u8]| {
        led::set_blinking(data[0] != 0);
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
                    return Err(AppError::BluetoothDisconnected);
                }
            }
            Err(err) => {
                println!("{:?}", err);
            }
        }
    }
}
