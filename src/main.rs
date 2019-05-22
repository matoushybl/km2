#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m as arm;
use stm32f0xx_hal as hal;
use cortex_m_rt as rt;

use arm::peripheral::{syst, Peripherals};
//use arm::interrupt::Mutex;
//
use hal::{prelude::*, stm32};
use hal::stm32::interrupt;
use hal::gpio::*;
use hal::gpio::gpioa::*;
use hal::gpio::gpiob::*;
use hal::gpio::gpiof::*;
use hal::time::KiloHertz;

use rt::{entry, exception};

use cortex_m_semihosting::hprintln;
//
//use core::cell::RefCell;
//use core::ops::DerefMut;
//
//use nb::*;
//
////mod i2c;
////use i2c::{I2c, SlaveAddress, SlaveRead};
//
//mod pin_defs;
//mod mcp4922;
//
//// temporary linking, move above if needed
//use arm::asm;
//use embedded_hal::spi::{MODE_0, MODE_3};
//use stm32f0xx_hal::delay::Delay;
//use crate::mcp4922::Channel;
//use core::borrow::{Borrow, BorrowMut};
//use core::ops::Deref;
//
//use heapless::Vec;
//use heapless::consts::U8;
//

//// static variables - data and peripherals used in interrupts
//static LED: Mutex<RefCell<Option<PA2<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
//static I2C_SLAVE: Mutex<RefCell<Option<stm32::i2c1::RegisterBlock>>> = Mutex::new(RefCell::new(None));
//static I2C_SLAVE_STATE: Mutex<RefCell<I2CSlaveState>> = Mutex::new(RefCell::new(I2CSlaveState::Idle));
//static I2C_REGISTER: Mutex<RefCell<u8>> = Mutex::new(RefCell::new(0));
//static I2C_BUFFER: Mutex<RefCell<Vec<u8, U8>>> = Mutex::new(RefCell::new(Vec::new()));
//static I2C_BUFFER_INDEX: Mutex<RefCell<u8>> = Mutex::new(RefCell::new(0));
//
//#[entry]
//fn main() -> ! {
//    let core_peripherals = Peripherals::take().unwrap();
//    let mut device_peripherals = stm32::Peripherals::take().unwrap();
//
//    let mut rcc = device_peripherals.RCC
//        .configure()
//        .sysclk(8.mhz())
//        .freeze(&mut device_peripherals.FLASH);
//
//    let gpioa: gpioa::Parts = device_peripherals.GPIOA.split(&mut rcc);
//    let gpiob: gpiob::Parts = device_peripherals.GPIOB.split(&mut rcc);
//    let gpiof: gpiof::Parts = device_peripherals.GPIOF.split(&mut rcc);
//
//    let mut scl: Option<PA9<Alternate<AF4>>> = None;
//    let mut sda: Option<PA10<Alternate<AF4>>> = None;
//
//    let mut sck: Option<PA5<Alternate<AF0>>> = None;
//    let mut cs_pin: Option<PA6<Output<PushPull>>> = None;
//    let mut mosi: Option<PA7<Alternate<AF0>>> = None;
//
//    arm::interrupt::free(|cs| {
//        scl.replace(gpioa.pa9.into_alternate_af4(cs));
//        sda.replace(gpioa.pa10.into_alternate_af4(cs));
//
//        sck.replace(gpioa.pa5.into_alternate_af0(cs));
//        mosi.replace(gpioa.pa7.into_alternate_af0(cs));
//        cs_pin.replace(gpioa.pa6.into_push_pull_output(cs));
//
//        LED.borrow(cs).replace(Some(gpioa.pa2.into_push_pull_output(cs)));
//    });
//
//    let mut systick = core_peripherals.SYST;
//    systick.set_clock_source(syst::SystClkSource::Core);
//    systick.set_reload(8 * 500 / 10 * cortex_m::peripheral::SYST::get_ticks_per_10ms());
//    systick.clear_current();
//    systick.enable_counter();
//    systick.enable_interrupt();
//
//
//    // enable RCC for i2c1
//    rcc.regs.apb1enr.write(|w| w.i2c1en().set_bit());
//    rcc.regs.apb1rstr.write(|w| w.i2c1rst().set_bit());
//    rcc.regs.apb1rstr.write(|w| w.i2c1rst().clear_bit());
//
//    let i2c_peripheral = device_peripherals.I2C1;
//
//    // Make sure the I2C unit is disabled so we can configure it
//    i2c_peripheral.cr1.modify(|_, w| w.pe().clear_bit());
//
//    // Calculate settings for I2C speed modes
//    let presc = 1;
//    let scldel = 4;
//    let sdadel = 2;
//    let scll = cmp::max((((8_000_000 >> presc) >> 1) / KiloHertz(100).0) - 1, 255) as u8;
//    let sclh = scll - 4;
//
//    // Enable I2C signal generator, and configure I2C for 400KHz full speed
//    i2c_peripheral.timingr.write(|w| {
//        w.presc()
//            .bits(presc as u8)
//            .scldel()
//            .bits(scldel)
//            .sdadel()
//            .bits(sdadel)
//            .sclh()
//            .bits(sclh)
//            .scll()
//            .bits(scll)
//    });
//
//    i2c_peripheral.oar1.modify(|r, w| {
//        w.oa1().bits(0x55 << 1);
//        w.oa1en().set_bit();
//        w.oa1mode().clear_bit()
//    });
//
//    // Enable the I2C processing
//    i2c_peripheral.cr1.write(|w|
//        w.pe()
//            .set_bit()
//            .addrie()
//            .set_bit()
//    );
//
//    loop {
//        asm::nop();
//    }
//}
//
//#[interrupt]
//fn I2C1() {
//    arm::interrupt::free(|cs| {
//        let peripheral = I2C_SLAVE.borrow(cs).borrow();
//        let interface_status = peripheral.as_ref().unwrap().isr.read();
//        let state = I2C_SLAVE_STATE.borrow(cs).borrow();
//
//        // handle errors, check for stop
//
//        match state.deref() {
//            I2CSlaveState::Idle => {
//                if interface_status.addr().bit_is_set() && interface_status.dir().bit_is_clear() {
//                    I2C_SLAVE_STATE.borrow(cs).replace(I2CSlaveState::Addressed);
//                    peripheral.as_ref().unwrap().icr.write(|w| w.addrcf().set_bit() );
//                }
//
//                I2C_BUFFER_INDEX.borrow(cs).replace(0);
//                I2C_BUFFER.borrow(cs).replace(Vec::new());
//            }
//            I2CSlaveState::Addressed => {
//                if interface_status.rxne().bit_is_set() {
//                    let register = peripheral.as_ref().unwrap().rxdr.read().bits() as u8;
//                    I2C_REGISTER.borrow(cs).replace(register);
//                    I2C_SLAVE_STATE.borrow(cs).replace(I2CSlaveState::RegisterSet);
//                }
//            }
//            I2CSlaveState::RegisterSet => {
//                if interface_status.addr().bit_is_set() && interface_status.dir().bit_is_set() {
//                    I2C_SLAVE_STATE.borrow(cs).replace(I2CSlaveState::Reading);
//                } else if interface_status.txe().bit_is_set() {
//                    I2C_SLAVE_STATE.borrow(cs).replace(I2CSlaveState::Writing);
////                    I2C1();
//                    // read first byte to trigger change in the state machine
//                    I2C_BUFFER_INDEX.borrow(cs).replace(1);
//                    I2C_BUFFER.borrow(cs).borrow_mut().deref_mut().push(1);
//                }
//            }
//            I2CSlaveState::Reading => {}
//            I2CSlaveState::Writing => {}
//            I2CSlaveState::Error(..) => {
//                // nack here?
//            }
//        }
//    });
//}
//
//#[exception]
//fn SysTick() {
//    arm::interrupt::free(|cs| {
//        if let &mut Some(ref mut led) = LED.borrow(cs).borrow_mut().deref_mut() {
//            led.toggle();
//        }
//    })
//}

use rtfm::app;

mod mcp4922;
mod pin_defs;

use core::cmp;

#[derive(Debug)]
pub enum I2CError {
    Overrun,
    ArbitrationLost,
    BusError,
    NACK,
}

pub enum I2CSlaveState {
    Idle,
    Addressed,
    RegisterSet,
    Reading,
    Writing,
    Error(I2CError),
}

#[app(device = stm32f0::stm32f0x1)]
const APP: () = {
    static mut DAC_DATA: [f32; 12] = [0f32, 0.3f32, 0.6f32, 0.9f32, 1.2f32, 1.5f32, 1.8f32, 2.1f32, 2.4f32, 2.7f32, 3f32, 3.3f32];
    static mut DAC_INDEX: u8 = 0;

    static mut I2C_SLAVE_STATE: I2CSlaveState = I2CSlaveState::Idle;
    static mut I2C_REGISTER: u8 = 0;
    static mut I2C_BUFFER: [u8; 8] = [0; 8];
    static mut I2C_BUFFER_INDEX: u8 = 0;

    static mut LED: PA2<Output<PushPull>> = ();
    static mut I2C_PERIPHERALS: stm32::I2C1 = ();
    static mut DAC: mcp4922::MCP4922<stm32::SPI1, PA5<Alternate<AF0>>, PA7<Alternate<AF0>>, PA6<Output<PushPull>>> = ();

    #[init]
    fn init() -> init::LateResources {
        let mut rcc = device.RCC
            .configure()
            .sysclk(8.mhz())
            .freeze(&mut device.FLASH);

        let mut systick = core.SYST;
        systick.set_clock_source(syst::SystClkSource::Core);
        systick.set_reload(8 * 500 / 10 * cortex_m::peripheral::SYST::get_ticks_per_10ms());
        systick.clear_current();
        systick.enable_counter();
        systick.enable_interrupt();

        let gpioa = device.GPIOA.split(&mut rcc);

        let mut sck = gpioa.pa5.into_alternate_af0();//: Option<PA5<Alternate<AF0>>> = None;
        let mut cs_pin = gpioa.pa6.into_push_pull_output();//: Option<PA6<Output<PushPull>>> = None;
        let mut mosi = gpioa.pa7.into_alternate_af0();//: Option<PA7<Alternate<AF0>>> = None;

        let scl = gpioa.pa9.into_alternate_af4();
        let sda = gpioa.pa10.into_alternate_af4();

        let mut dac = mcp4922::MCP4922::initialize(
            device.SPI1,
            sck,
            mosi,
            cs_pin,
            &mut rcc);

        // Make sure the I2C unit is disabled so we can configure it
        device.I2C1.cr1.modify(|_, w| w.pe().clear_bit());

        // Calculate settings for I2C speed modes
        let scll = cmp::max((((8_000_000 >> 1) >> 1) / KiloHertz(100).0) - 1, 255) as u8;

        // Enable I2C signal generator, and configure I2C for 400KHz full speed
        device.I2C1.timingr.write(|w| {
            w.presc()
                .bits(1)
                .scldel()
                .bits(4)
                .sdadel()
                .bits(2)
                .sclh()
                .bits(scll - 4)
                .scll()
                .bits(scll)
        });

        device.I2C1.oar1.write(|w| w.oa1().bits(0x55 << 1).oa1en().set_bit().oa1mode().clear_bit());

        // Enable the I2C processing
        device.I2C1.cr1.write(|w| w.pe().set_bit().addrie().set_bit());

        init::LateResources {
            LED: gpioa.pa2.into_push_pull_output(),
            DAC: dac,
            I2C_PERIPHERALS: device.I2C1,
        }
    }

    #[exception(resources = [LED, DAC, DAC_DATA, DAC_INDEX])]
    fn SysTick() {
        resources.LED.toggle();
        resources.DAC.set_voltage(mcp4922::Channel::A, resources.DAC_DATA[*resources.DAC_INDEX as usize]);

        if *resources.DAC_INDEX == 11 {
            *resources.DAC_INDEX = 0;
        } else {
            *resources.DAC_INDEX += 1;
        }
    }

    #[interrupt(resources = [I2C_PERIPHERALS, I2C_SLAVE_STATE, I2C_REGISTER, I2C_BUFFER, I2C_BUFFER_INDEX])]
    fn I2C1() {
        let interface_status = (*resources.I2C_PERIPHERALS).isr.read();
        // handle errors, check for stop

        match *resources.I2C_SLAVE_STATE {
            I2CSlaveState::Idle => {
                if interface_status.addr().bit_is_set() && interface_status.dir().bit_is_clear() {
                    *resources.I2C_SLAVE_STATE = I2CSlaveState::Addressed;
                    (*resources.I2C_PERIPHERALS).icr.write(|w| w.addrcf().set_bit());
                }

                *resources.I2C_BUFFER_INDEX = 0;
                *resources.I2C_BUFFER = [0; 8];
            }
            I2CSlaveState::Addressed => {
                if interface_status.rxne().bit_is_set() {
                    let register = (*resources.I2C_PERIPHERALS).rxdr.read().bits() as u8;
                    *resources.I2C_REGISTER = register;
                    *resources.I2C_SLAVE_STATE = I2CSlaveState::RegisterSet;
                }
            }
            I2CSlaveState::RegisterSet => {
                if interface_status.addr().bit_is_set() && interface_status.dir().bit_is_set() {
                    *resources.I2C_SLAVE_STATE = I2CSlaveState::Reading;
                } else if interface_status.txe().bit_is_set() {
                    *resources.I2C_SLAVE_STATE = I2CSlaveState::Writing;
//                    I2C1();
                    // read first byte to trigger change in the state machine
                    *resources.I2C_BUFFER_INDEX = 1;
                    resources.I2C_BUFFER[0] = (*resources.I2C_PERIPHERALS).txdr.read().bits() as u8;
                }
            }
            I2CSlaveState::Reading => {}
            I2CSlaveState::Writing => {}
            I2CSlaveState::Error(..) => {
                // nack here?
            }
        }
    }
};