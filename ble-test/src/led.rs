//! 
//! 
//! 
use core::{cell::RefCell, sync::atomic::{AtomicBool, Ordering}};
use critical_section::Mutex;
use esp_hal::{delay::MicrosDurationU64, gpio::{GpioPin, Level, Output}, interrupt::{self, Priority}, ledc, macros::handler, peripherals::{Interrupt, TIMG1}, timer::timg::{Timer, Timer0, TimerGroup}, Blocking};
use esp_hal::prelude::*;
use crate::Global;

static TIMER0: Global<Timer<Timer0<TIMG1>, Blocking>> = Mutex::new(RefCell::new(None));

static LED: Global<Output<'static>> = Mutex::new(RefCell::new(None));

static BLINKING: AtomicBool = AtomicBool::new(false);

pub fn setup(pin: GpioPin<8>, timer_group: TIMG1)
{
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

pub fn get_blinking() -> bool {
    BLINKING.load(Ordering::Relaxed)
}

pub fn set_blinking(blink: bool) {
    if blink {
        if !BLINKING.load(Ordering::Relaxed) {

            BLINKING.store(true, Ordering::Relaxed);
            critical_section::with(|cs| {
                let mut timer0 = TIMER0.borrow_ref_mut(cs);
                let timer0 = timer0.as_mut().unwrap();
                if timer0.has_elapsed() {
                    timer0.load_value(500u64.millis()).unwrap();
                    timer0.start();
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
        let timer0 = timer0.as_mut().unwrap();
        timer0.clear_interrupt();

        let mut led = LED.borrow_ref_mut(cs);
        let led = led.as_mut().unwrap();
        
        if BLINKING.load(Ordering::Relaxed) {
            led.toggle();
                
            timer0.load_value(250u64.millis()).unwrap();
            timer0.start();
        } 
        else {
            led.set_low();
        }
    });
}