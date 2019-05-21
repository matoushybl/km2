#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m as arm;
use stm32f0xx_hal as hal;
use cortex_m_rt as rt;

use arm::peripheral::{syst, Peripherals};
use arm::interrupt::{Mutex};

use hal::{prelude::*, stm32};
use hal::stm32::interrupt;
use hal::gpio::*;
use hal::gpio::gpioa::*;
use hal::gpio::gpiob::*;
use hal::gpio::gpiof::*;
use hal::time::KiloHertz;

use rt::{entry, exception};

use core::cell::RefCell;
use core::ops::DerefMut;

use nb::*;

mod i2c;
use i2c::{I2c, SlaveAddress, SlaveRead};

mod pin_defs;
mod mcp4922;

// temporary linking, move above if needed
use arm::asm;
use embedded_hal::spi::{MODE_0, MODE_3};
use stm32f0xx_hal::delay::Delay;
use crate::mcp4922::Channel;

// static variables - data and peripherals used in interrupts
static LED: Mutex<RefCell<Option<PA2<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let core_peripherals = Peripherals::take().unwrap();
    let mut device_peripherals = stm32::Peripherals::take().unwrap();

    let mut rcc = device_peripherals.RCC
        .configure()
        .sysclk(8.mhz())
        .freeze(&mut device_peripherals.FLASH);

    let gpioa: gpioa::Parts = device_peripherals.GPIOA.split(&mut rcc);
    let gpiob: gpiob::Parts = device_peripherals.GPIOB.split(&mut rcc);
    let gpiof: gpiof::Parts = device_peripherals.GPIOF.split(&mut rcc);

    let mut scl: Option<PA9<Alternate<AF4>>> = None;
    let mut sda: Option<PA10<Alternate<AF4>>> = None;

    let mut sck: Option<PA5<Alternate<AF0>>> = None;
    let mut cs_pin: Option<PA6<Output<PushPull>>> = None;
    let mut mosi: Option<PA7<Alternate<AF0>>> = None;

    arm::interrupt::free( |cs| {
        scl.replace(gpioa.pa9.into_alternate_af4(cs));
        sda.replace(gpioa.pa10.into_alternate_af4(cs));

        sck.replace(gpioa.pa5.into_alternate_af0(cs));
        mosi.replace(gpioa.pa7.into_alternate_af0(cs));
        cs_pin.replace(gpioa.pa6.into_push_pull_output(cs));

        LED.borrow(cs).replace(Some(gpioa.pa2.into_push_pull_output(cs)));
    });

//    let mut systick = core_peripherals.SYST;
//    systick.set_clock_source(syst::SystClkSource::Core);
//    systick.set_reload(8 * 500 / 10 * cortex_m::peripheral::SYST::get_ticks_per_10ms());
//    systick.clear_current();
//    systick.enable_counter();
//    systick.enable_interrupt();

    let mut i2c_slave = I2c::i2c1(device_peripherals.I2C1,
                                  (scl.unwrap(), sda.unwrap()),
                                  KiloHertz(100),
                                  &mut rcc).into_slave();

    i2c_slave.set_own_slave_addr(0x55);

    let mut delay = Delay::new(core_peripherals.SYST, &rcc);

    let mut dac = mcp4922::MCP4922::initialize(
        device_peripherals.SPI1,
        sck.unwrap(),
        mosi.unwrap(),
        cs_pin.unwrap(),
        &mut rcc);

    let data: [f32; 10] = [ 0.0, 0.5, 1.0, 1.5, 1.8, 2.4, 2.8, 3.0, 3.1, 3.3];
    loop {
        for &value in &data {
            dac.set_voltage(Channel::A, value);
            dac.set_voltage(Channel::B, value);

            delay.delay_ms(10_u16);
        }
    }
}

#[exception]
fn SysTick() {
    arm::interrupt::free(|cs| {
        if let &mut Some(ref mut led) = LED.borrow(cs).borrow_mut().deref_mut() {
            led.toggle();
        }
    })
}

enum I2CSlaveState {
    Idle,
    Addressed,
    RegisterSet,
    Reading,
    Writing,
    Error,
}

#[interrupt]
fn I2C1() {

}