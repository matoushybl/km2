#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m as arm;
use stm32f0xx_hal as hal;

use hal::{prelude::*, stm32};
use hal::gpio::*;
use hal::gpio::gpioa::*;
use hal::gpio::gpiof::*;

use rtfm::app;

use arm::peripheral::syst;

mod mcp4922;
mod pin_defs;

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
    ShouldReceive,
    ShouldTransmit,
    Error(I2CError),
}

#[app(device = stm32f0::stm32f0x1)]
const APP: () = {
//    static mut I2C_SLAVE_STATE: I2CSlaveState = I2CSlaveState::Idle;
//    static mut I2C_REGISTER: u8 = 0;
//    static mut I2C_BUFFER: [u8; 8] = [0; 8];
//    static mut I2C_BUFFER_INDEX: u8 = 0;

    static mut TIMER1: stm32f0::stm32f0x1::TIM16 = ();
//    static mut TIMER2: stm32f0::stm32f0x1::TIM17 = ();

    static mut LED: PA2<Output<PushPull>> = ();

    static mut EN: PF1<Output<PushPull>> = ();
    static mut FAULT: PA1<Input<PullUp>> = ();
//
    static mut M1STEP: PF0<Output<PushPull>> = ();
    static mut M1DIR: PA0<Output<PushPull>> = ();

    static mut M2STEP: PA3<Output<PushPull>> = ();
    static mut M2DIR: PA4<Output<PushPull>> = ();

//    static mut I2C_PERIPHERALS: stm32f0::stm32f0x1::I2C1 = ();
    static mut DAC: mcp4922::MCP4922<stm32::SPI1, PA5<Alternate<AF0>>, PA7<Alternate<AF0>>, PA6<Output<PushPull>>> = ();

    #[init]
    fn init() -> init::LateResources {
        let mut rcc = device.RCC
            .configure()
            .sysclk(24.mhz())
            .pclk(24.mhz())
            .freeze(&mut device.FLASH);

//        let mut systick = core.SYST;
//        systick.set_clock_source(syst::SystClkSource::Core);
//        systick.set_reload(8 * 500 / 10 * cortex_m::peripheral::SYST::get_ticks_per_10ms());
//        systick.clear_current();
//        systick.enable_counter();
//        systick.enable_interrupt();

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiof = device.GPIOF.split(&mut rcc);

        let sck = gpioa.pa5.into_alternate_af0();//: Option<PA5<Alternate<AF0>>> = None;
        let cs_pin = gpioa.pa6.into_push_pull_output();//: Option<PA6<Output<PushPull>>> = None;
        let mosi = gpioa.pa7.into_alternate_af0();//: Option<PA7<Alternate<AF0>>> = None;

        let scl = gpioa.pa9.into_alternate_af4();
        let sda = gpioa.pa10.into_alternate_af4();

        let mut dac = mcp4922::MCP4922::initialize(
            device.SPI1,
            sck,
            mosi,
            cs_pin,
            &mut rcc);

        dac.set_voltage(mcp4922::Channel::A, 0.3);
        dac.set_voltage(mcp4922::Channel::B, 0.3);

        rcc.regs.apb1enr.modify(|_, w| w.i2c1en().set_bit());
         // Reset I2C
        rcc.regs.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
        rcc.regs.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());

        // Make sure the I2C unit is disabled so we can configure it
//        device.I2C1.cr1.modify(|_, w| w.pe().clear_bit());

        // Calculate settings for I2C speed modes
//        let scll = cmp::max((((8_000_000 >> 1) >> 1) / KiloHertz(100).0) - 1, 255) as u8;
//
//        // Enable I2C signal generator, and configure I2C for 400KHz full speed
//        device.I2C1.timingr.write(|w| {
//            w.presc()
//                .bits(1)
//                .scldel()
//                .bits(4)
//                .sdadel()
//                .bits(2)
//                .sclh()
//                .bits(scll - 4)
//                .scll()
//                .bits(scll)
//        });
//
//        device.I2C1.oar1.write(|w| w.oa1().bits(0x55 << 1).oa1en().set_bit().oa1mode().clear_bit());
//
//        // Enable the I2C processing
//        device.I2C1.cr1.write(|w|
//            w.pe().set_bit()
//                .errie().set_bit()
//                .stopie().set_bit()
//                .nackie().set_bit()
//                .addrie().set_bit()
//                .rxie().set_bit()
//                .nostretch().set_bit()
//                .txie().set_bit()
//        );
//
//        device.I2C1.icr.write(|w|
//            w.stopcf().set_bit()
//                .nackcf().set_bit()
//                .ovrcf().set_bit()
//                .arlocf().set_bit()
//                .berrcf().set_bit()
//        );

        let timer1: stm32f0::stm32f0x1::TIM16 = device.TIM16;

        // enable clock
        rcc.regs.apb2enr.write(|w| w.tim16en().set_bit());
        // Reset I2C
        rcc.regs.apb2rstr.write(|w| w.tim16rst().set_bit());
        rcc.regs.apb2rstr.write(|w| w.tim16rst().clear_bit());

        let frequency = 1000;

        timer1.cr1.write(|w| w.cen().clear_bit());
        timer1.egr.write(|w| w.ug().set_bit());
        timer1.dier.write(|w| w.uie().set_bit());
        timer1.cnt.reset();

        let psc = 8_000_000 / frequency - 1;
        timer1.psc.write(|w| unsafe { w.bits(24000) });
        timer1.arr.write(|w| unsafe { w.bits(1)});

        timer1.sr.write(|w| w.uif().clear_bit().bif().clear_bit().cc1if().clear_bit().cc1of().clear_bit().comif().clear_bit().tif().clear_bit());
        timer1.cr1.write(|w| w.cen().set_bit());


//        let timer2: stm32f0::stm32f0x1::TIM17 = device.TIM17;
//
//        // enable clock
//        rcc.regs.apb2enr.write(|w| w.tim17en().set_bit());
//        // Reset I2C
//        rcc.regs.apb2rstr.write(|w| w.tim17rst().set_bit());
//        rcc.regs.apb2rstr.write(|w| w.tim17rst().clear_bit());
//
//        timer2.cr1.write(|w| w.cen().clear_bit());
//        timer2.egr.write(|w| w.ug().set_bit());
//        timer2.dier.write(|w| w.uie().set_bit());
//        timer2.cnt.reset();
//
//        timer2.psc.write(|w| unsafe { w.bits(24000) });
//        timer2.arr.write(|w| unsafe { w.bits(1)});
//
//        timer2.sr.write(|w| w.uif().clear_bit().bif().clear_bit().cc1if().clear_bit().cc1of().clear_bit().comif().clear_bit().tif().clear_bit());
//        timer2.cr1.write(|w| w.cen().set_bit());

        init::LateResources {
            LED: gpioa.pa2.into_push_pull_output(),
            EN: gpiof.pf1.into_push_pull_output(),
            FAULT: gpioa.pa1.into_pull_up_input(),
            M1STEP: gpiof.pf0.into_push_pull_output(),
            M1DIR: gpioa.pa0.into_push_pull_output(),
            M2STEP: gpioa.pa3.into_push_pull_output(),
            M2DIR: gpioa.pa4.into_push_pull_output(),
            DAC: dac,
//            I2C_PERIPHERALS: device.I2C1,
            TIMER1: timer1,
//            TIMER2: timer2,
        }
    }

    #[interrupt(resources = [LED, TIMER1, M1STEP])]
    fn TIM16() {
        let status = resources.TIMER1.sr.read();
        if status.uif().bit_is_set() {
            resources.M1STEP.toggle();

            resources.TIMER1.sr.write(|w| w.uif().clear_bit().bif().clear_bit().cc1if().clear_bit().cc1of().clear_bit().comif().clear_bit().tif().clear_bit());
        }
        resources.TIMER1.sr.write(|w| w.uif().clear_bit().bif().clear_bit().cc1if().clear_bit().cc1of().clear_bit().comif().clear_bit().tif().clear_bit());
    }

//    #[interrupt(resources = [TIMER2, M2STEP])]
//    fn TIM17() {
//        if resources.TIMER2.sr.read().uif().bit_is_set() {
//            resources.M2STEP.toggle();
//            resources.TIMER2.sr.write(|w| w.uif().clear_bit());
//        }
//        resources.TIMER2.sr.write(|w| w.uif().clear_bit().bif().clear_bit().cc1if().clear_bit().cc1of().clear_bit().comif().clear_bit().tif().clear_bit());
//    }

//    #[idle(resources = [I2C_SLAVE_STATE])]
//    fn idle() -> ! {
//        loop {
////            match resources.I2C_SLAVE_STATE {
////                I2CSlaveState::Error(I2CError::NACK) => {
////                    hprintln!("Error: NACK");
////                },
////                _ => {}
////            }
//        }
//    }

//    #[exception(resources = [LED])]
//    fn SysTick() {
//        resources.LED.toggle();
//    }
//
//    #[interrupt(resources = [I2C_PERIPHERALS, I2C_SLAVE_STATE, I2C_REGISTER, I2C_BUFFER, I2C_BUFFER_INDEX])]
//    fn I2C1() {
//        let interface_status = (*resources.I2C_PERIPHERALS).isr.read();
//        // handle errors, check for stop
//
//        if interface_status.nackf().bit_is_set() {
//            (*resources.I2C_PERIPHERALS).icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
//            *resources.I2C_SLAVE_STATE = I2CSlaveState::Idle;
//            return;
//        } else if interface_status.ovr().bit_is_set() {
//            (*resources.I2C_PERIPHERALS).icr.write(|w| w.stopcf().set_bit().ovrcf().set_bit());
//            *resources.I2C_SLAVE_STATE = I2CSlaveState::Idle;
//            return;
//        } else if interface_status.arlo().bit_is_set() {
//            (*resources.I2C_PERIPHERALS).icr.write(|w| w.stopcf().set_bit().arlocf().set_bit());
//            *resources.I2C_SLAVE_STATE = I2CSlaveState::Idle;
//
//            return;
//        } else if interface_status.berr().bit_is_set() {
//            (*resources.I2C_PERIPHERALS).icr.write(|w| w.stopcf().set_bit().berrcf().set_bit());
//            *resources.I2C_SLAVE_STATE = I2CSlaveState::Idle;
//
//            return;
//        }
//
//        // check for stop, go to idle, reset buffers
//        if interface_status.stopf().bit_is_set() {
//            (*resources.I2C_PERIPHERALS).icr.write(|w| w.stopcf().set_bit());
//            *resources.I2C_SLAVE_STATE = I2CSlaveState::Idle;
////            hprintln!("STOP -> IDLE");
////            hprintln!("register: {}", *resources.I2C_REGISTER);
////            hprintln!("bytes read: {}, bytes: {}", *resources.I2C_BUFFER_INDEX, (*resources.I2C_BUFFER)[0]);
//            *resources.I2C_BUFFER_INDEX = 0;
//            *resources.I2C_REGISTER = 0;
//            *resources.I2C_BUFFER = [0; 8];
//            return;
//        }
//
//        match *resources.I2C_SLAVE_STATE {
//            I2CSlaveState::Idle => {
//                if interface_status.addr().bit_is_set() && interface_status.dir().bit_is_clear() {
//                    (*resources.I2C_PERIPHERALS).icr.write(|w| w.addrcf().set_bit());
//                    *resources.I2C_SLAVE_STATE = I2CSlaveState::Addressed;
////                    hprintln!("a");
////                    hprintln!("IDLE -> ADDR");
//                    if interface_status.rxne().bit_is_set() {
//                        let register = (*resources.I2C_PERIPHERALS).rxdr.read().bits() as u8;
//                        *resources.I2C_REGISTER = register;
//                        *resources.I2C_SLAVE_STATE = I2CSlaveState::RegisterSet;
////                    hprintln!("ADDR -> REG_SET");
////                    hprintln!("rs");
//                    }
//                }
//
//                *resources.I2C_BUFFER_INDEX = 0;
//                *resources.I2C_BUFFER = [0; 8];
//            }
//            I2CSlaveState::Addressed => {
//                if interface_status.rxne().bit_is_set() {
//                    let register = (*resources.I2C_PERIPHERALS).rxdr.read().bits() as u8;
//                    *resources.I2C_REGISTER = register;
//                    *resources.I2C_SLAVE_STATE = I2CSlaveState::RegisterSet;
////                    hprintln!("ADDR -> REG_SET");
////                    hprintln!("rs");
//                }
//            }
//            I2CSlaveState::RegisterSet => {
//                if interface_status.dir().bit_is_clear() {
//                    *resources.I2C_SLAVE_STATE = I2CSlaveState::ShouldReceive;
////                    hprintln!("REG_SET -> SHOULD_RX");
////                    if interface_status.rxne().bit_is_set() {
////                        // read first byte to trigger change in the state machine
//////                        hprintln!("byte: {} read", *resources.I2C_BUFFER_INDEX);
////                        *resources.I2C_BUFFER_INDEX = 1;
////                        resources.I2C_BUFFER[0] = (*resources.I2C_PERIPHERALS).rxdr.read().bits() as u8;
////                    }
//                } else if interface_status.addr().bit_is_set() && interface_status.dir().bit_is_set() {
//                    (*resources.I2C_PERIPHERALS).icr.write(|w| w.addrcf().set_bit());
//                    *resources.I2C_BUFFER = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
//                    *resources.I2C_SLAVE_STATE = I2CSlaveState::ShouldTransmit;
////                    hprintln!("REG_SET -> SHOULD_TX");
//                }
//            }
//            I2CSlaveState::ShouldReceive => {
//                if interface_status.rxne().bit_is_set() {
////                    hprintln!("byte: {} read", *resources.I2C_BUFFER_INDEX);
//                    resources.I2C_BUFFER[*resources.I2C_BUFFER_INDEX as usize] = (*resources.I2C_PERIPHERALS).rxdr.read().bits() as u8;
//                    *resources.I2C_BUFFER_INDEX += 1;
//                }
//            }
//            I2CSlaveState::ShouldTransmit => {
//                if interface_status.txe().bit_is_set() {
//                    (*resources.I2C_PERIPHERALS).txdr.write(|w| w.txdata().bits((*resources.I2C_BUFFER)[*resources.I2C_BUFFER_INDEX as usize]));
//                    *resources.I2C_BUFFER_INDEX += 1;
//                }
//            }
//            I2CSlaveState::Error(..) => {
////                hprintln!("ERR - forbidden state");
//                // nack here?
//            }
//        }
//    }
};
