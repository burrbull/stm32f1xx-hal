//! Testing PWM output for custom pin combinations

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // TIM3
    let p0 = pb4.into_alternate_push_pull(&mut gpiob.crl);
    let p1 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let pwm = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1).pwm((p0, p1), &mut afio.mapr, 1.khz());

    let max = pwm.get_max_duty().unwrap();

    let mut pwm_channels = pwm.split();

    // Enable the individual channels
    pwm_channels.0.enable().unwrap();
    pwm_channels.1.enable().unwrap();

    // full
    pwm_channels.0.set_duty(max).unwrap();
    pwm_channels.1.set_duty(max).unwrap();

    asm::bkpt();

    // dim
    pwm_channels.1.set_duty(max / 4).unwrap();

    asm::bkpt();

    // zero
    pwm_channels.0.set_duty(0).unwrap();
    pwm_channels.1.set_duty(0).unwrap();

    asm::bkpt();

    loop {}
}
