
use core::sync::atomic::Ordering;
use esp_hal::time;
use crate::{led, sensor::ANALYZING, step::Step, STEPS_QUEUE};

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

/// Checks if the device is currently analyzing the motion for steps. Value returned as a 1 or 0 in
/// the first byte of the sent data.
/// 
pub fn get_analyzing(_offset: usize, data: &mut [u8]) -> usize {
    data[0] = if ANALYZING.load(Ordering::Relaxed) { 0x01 } else { 0x00 };
    1
}

/// Enable/disable the analyzing of motion for steps. If the first byte of the sent data is set to
/// 0 it will stop analyzing, otherwise it will enable the analyzing.
/// 
pub fn set_analyzing(_offset: usize, data: &[u8]) {
    ANALYZING.store(data[0] != 0, Ordering::Relaxed);
}

/// Gets the current length of the step detection queue and one element from it (if it contains 
/// one). Bytes are layed out in the following order:
/// 
/// [length][step start (micro seconds)    ][step end (micro seconds)      ]
/// 0       1       2       3       4       5       6       7       8       
/// 
/// Note that the length is taken before the step is removed from the queue (so if the returned
/// length is 0, the following bytes do not contain a step and if the returned length is 3, there
/// are 2 steps left in the queue on the ESP32).
/// 
/// 
pub fn get_detection_queue(_offset: usize, data: &mut [u8]) -> usize {
    let mut size = 0u8;
    let mut step: Option<Step> = None;
    critical_section::with(|cs| {
        if let Some(queue) = STEPS_QUEUE.borrow(cs).borrow_mut().as_mut() {
            size = queue.len() as u8;
            step = queue.next();
        }
    });
    data[0] = size;
    if let Some(step) = step {
        data[1..5].copy_from_slice(&step.start.to_be_bytes());
        data[5..9].copy_from_slice(&step.stop.to_be_bytes());
        9
    } 
    else {
        1
    }
}
