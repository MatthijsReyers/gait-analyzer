//! This module manages the ESP32-C3 mini's built-in LED so it can easily be used for debugging
//! purposes. Should the LED's pin already be in use for something else, the `setup` function can
//! simply be skipped causing the `get_blinking` and `set_blinking` functions to essentially do
//! nothing.
//! 
use core::{cell::RefCell, sync::atomic::{AtomicBool, Ordering}};
use critical_section::Mutex;
use esp_hal::prelude::*;
use crate::Global;
use esp_hal::{
    gpio::{GpioPin, Level, Output}, 
    interrupt::{self, Priority}, 
    peripherals::{Interrupt, TIMG1}, 
    timer::timg::{Timer, Timer0, TimerGroup},
    Blocking
};

/// Timer used for periodic interrupts in which we set the LED high and low.
static TIMER0: Global<Timer<Timer0<TIMG1>, Blocking>> = Mutex::new(RefCell::new(None));

/// LED output pin.
static LED: Global<Output<'static>> = Mutex::new(RefCell::new(None));

/// Is the LED currently blinking?
static BLINKING: AtomicBool = AtomicBool::new(false);

/// Setup all the hardware and interrupts that the LED needs to blink.
/// 
pub fn setup(pin: GpioPin<8>, timer_group: TIMG1) {
    let led = Output::new(pin, Level::High);
    
    let timers = TimerGroup::new(timer_group);
    let timer0 = timers.timer0;
    timer0.set_interrupt_handler(on_timer_interrupt);

    interrupt::enable(Interrupt::TG1_T0_LEVEL, Priority::Priority1).unwrap();
    timer0.load_value(250u64.millis()).unwrap();
    timer0.start();
    timer0.listen();
    
    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
        LED.borrow_ref_mut(cs).replace(led);
    });
}

/// Gets the blinking state of the LED, i.e is the LED currently blinking? (And NOT is the led
/// currently on!).
/// 
pub fn get_blinking() -> bool {
    BLINKING.load(Ordering::Relaxed)
}

/// Set the blinking state of the LED
/// 
pub fn set_blinking(blink: bool) {
    if blink {
        if !BLINKING.load(Ordering::Relaxed) {

            BLINKING.store(true, Ordering::Relaxed);
            critical_section::with(|cs| {
                let mut timer0 = TIMER0.borrow_ref_mut(cs);
                if let Some(timer0)  = timer0.as_mut() {
                    if timer0.has_elapsed() {
                        timer0.load_value(500u64.millis()).unwrap();
                        timer0.start();
                    }
                }
            });
        }
    }
    else {
        BLINKING.store(true, Ordering::Relaxed);
    }
}

#[handler]
fn on_timer_interrupt() {
    critical_section::with(|cs| {

        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        if let Some(timer0) = timer0.as_mut() {

            timer0.clear_interrupt();
    
            let mut led = LED.borrow_ref_mut(cs);
            if let Some(led) = led.as_mut() {
    
                if BLINKING.load(Ordering::Relaxed) {
                    led.toggle();
                        
                    timer0.load_value(250u64.millis()).unwrap();
                    timer0.start();
                } 
                else {
                    led.set_low();
                }
            }
        }
    });
}