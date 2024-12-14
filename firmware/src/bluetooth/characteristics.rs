
use esp_hal::time;
use crate::led;

/// Get the current ESP32 system time. That is to say; microseconds passed since boot, as a u64 in
/// in big-endian byte order.
/// 
pub fn get_sys_time(_offset: usize, data: &mut [u8]) -> usize {
    let now = time::now().duration_since_epoch().to_micros();
    data[..8].copy_from_slice(&now.to_be_bytes());
    8
}

/// Get the current blinking state of the LED, this will return 1 in the first byte of the sent
/// data if the LED is currently blinking and 0 otherwise.
/// 
pub fn get_blink_led(_offset: usize, data: &mut [u8]) -> usize {
    data[0] = if led::get_blinking() { 0xFF } else { 0x00 };
    1
}

/// Sets the blinking state of the LED, if the first byte of the sent data is 0 it will stop
/// blinking, otherwise it will enable the blinking.
/// 
pub fn set_blink_led(_offset: usize, data: &[u8]) {
    log::info!("LED blink: {}", data[0]);
    led::set_blinking(data[0] != 0);
}

