//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use nb::block;

use svisual::{NextValue, SVMap, SendPackage, SetValue};

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    delay::Delay,
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // USART3
    // Configure pb10 as a push_pull output, this will be the tx pin
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // Take ownership over pb11
    let rx = gpiob.pb11;

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let (mut tx, _) = Serial::usart3(
        p.USART3,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    let mut delay = Delay::new(cp.SYST, clocks);

    // Create new map with not more than 2 different signals and 10 values in package
    let mut sv_map = SVMap::<2, 10>::new();

    loop {
        for i in 0..30 {
            // Set value of first signal of integers
            sv_map.set("temp", 15 + i).ok();
            // Set value of second signal of floats
            sv_map.set("temp2", 14. - (i as f32) / 2.).ok();
            // Use next value cell
            sv_map.next(|s| {
                // if package is full, send package with module name
                block!(tx.send_package("TempMod", s)).ok();
            });
            // Wait
            delay.delay_ms(100u16);
        }
    }
}
