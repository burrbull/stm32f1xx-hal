#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::{InputPin, OutputPin};
use stm32f1xx_hal::{gpio::State, pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut pin = gpioc.pc13.into_floating_input(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks)
        .start_count_down(1.hz())
        .unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        hprintln!("{}", pin.is_high().unwrap()).unwrap();
        pin.as_push_pull_output(&mut gpioc.crh, |out| {
            out.set_high().unwrap();
            block!(timer.wait()).unwrap();
            out.set_low().unwrap();
            block!(timer.wait()).unwrap();
        });
        pin.as_push_pull_output_with_state(&mut gpioc.crh, State::High, |out| {
            block!(timer.wait()).unwrap();
            out.set_low().unwrap();
            block!(timer.wait()).unwrap();
        });
    }
}
