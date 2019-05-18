#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m as arm;
use stm32f0xx_hal as hal;
use cortex_m_rt as rt;

use arm::peripheral::{syst, Peripherals};
use arm::interrupt::{Mutex};

use hal::{prelude::*, stm32};
use hal::gpio::{GpioExt, Output, PushPull, AF0, AF4, Alternate, gpioa::{PA5, PA6, PA7}};
use hal::time::KiloHertz;

use rt::{entry, exception};

use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m_semihosting::{hprintln};

use nb::*;

mod i2c;
use i2c::{I2c, SlaveAddress, SlaveRead};

mod spi;
use spi::{SPI, SimplexWrite};

// temporary linking, move above if needed
use arm::asm;
use embedded_hal::spi::MODE_0;
use crate::spi::{Send, NonDirectional};

// static variables - data and peripherals used in interrupts
static LED: Mutex<RefCell<Option<hal::gpio::gpioa::PA2<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

    let core_peripherals = Peripherals::take().unwrap();
    let mut device_peripherals = stm32::Peripherals::take().unwrap();

    let mut rcc = device_peripherals.RCC
        .configure()
        .sysclk(8.mhz())
        .freeze(&mut device_peripherals.FLASH);

    let gpioa: hal::gpio::gpioa::Parts = device_peripherals.GPIOA.split(&mut rcc);
    let gpiob: hal::gpio::gpiob::Parts = device_peripherals.GPIOB.split(&mut rcc);
    let gpiof: hal::gpio::gpiof::Parts = device_peripherals.GPIOF.split(&mut rcc);

    let mut scl: Option<hal::gpio::gpioa::PA9<Alternate<AF4>>> = None;
    let mut sda: Option<hal::gpio::gpioa::PA10<Alternate<AF4>>> = None;

    let mut sck: Option<hal::gpio::gpioa::PA5<Alternate<AF0>>> = None;
    let mut miso: Option<hal::gpio::gpioa::PA6<Alternate<AF0>>> = None;
    let mut mosi: Option<hal::gpio::gpioa::PA7<Alternate<AF0>>> = None;

    arm::interrupt::free( |cs| {
        scl.replace(gpioa.pa9.into_alternate_af4(cs));
        sda.replace(gpioa.pa10.into_alternate_af4(cs));
        LED.borrow(cs).replace(Some(gpioa.pa2.into_push_pull_output(cs)));
    });

    let mut systick = core_peripherals.SYST;
    systick.set_clock_source(syst::SystClkSource::Core);
    systick.set_reload(8 * 500 / 10 * cortex_m::peripheral::SYST::get_ticks_per_10ms());
    systick.clear_current();
    systick.enable_counter();
    systick.enable_interrupt();

    let mut i2c_slave = I2c::i2c1(device_peripherals.I2C1,
                                  (scl.unwrap(), sda.unwrap()),
                                  KiloHertz(100),
                                  &mut rcc).into_slave();

    i2c_slave.set_own_slave_addr(0x55);

    let spi: SPI<
        hal::stm32::SPI1,
        PA5<Alternate<AF0>>,
        PA6<Alternate<AF0>>,
        PA7<Alternate<AF0>>,
        NonDirectional> = SPI::spi1(
        device_peripherals.SPI1,
        sck.unwrap(),
        MODE_0,
        1.mhz(),
        16,
        false,
        &mut rcc);


    //.into_write_only(mosi.unwrap())
    //spi.send(0xffff);

    loop {
        hprintln!("waiting").unwrap();
        block!(i2c_slave.wait_for_addressing());

        let mut buffer: [u8; 5] = [0; 5];

        let res1 = i2c_slave.read(&mut buffer);
        match res1 {
            Ok(..) => hprintln!("byte read {:?}", buffer).unwrap(),
            Err(..) => hprintln!("read failed").unwrap()
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