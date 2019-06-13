use stm32f0xx_hal as hal;

use hal::rcc::Rcc;

use crate::pin_defs::{ SckPin, MosiPin };

use core::ops::Deref;

use nb::*;

use cortex_m::asm;
use embedded_hal::spi::{MODE_0, Phase, Polarity};
use embedded_hal::digital::OutputPin;
use core::cmp::{min, max};

pub enum Channel {
    A, B
}

pub struct MCP4922<SPI, SCK, MOSI, CS> {
    spi: SPI,
    sck: SCK,
    mosi: MOSI,
    cs: CS
}

#[allow(dead_code)]
type SpiRegisterBlock = hal::stm32::spi1::RegisterBlock;

impl<SPI, SCK, MOSI, CS> MCP4922<SPI, SCK, MOSI, CS> where
    SPI: Deref<Target = SpiRegisterBlock>,
    CS: OutputPin {

    pub fn initialize(spi: SPI, sck: SCK, mosi: MOSI, cs: CS, rcc: &mut Rcc) -> Self
        where
        SCK: SckPin<SPI>,
        MOSI: MosiPin<SPI>
    {

        rcc.regs.apb2enr.modify(|_, w| w.spi1en().set_bit());

        /* Reset SPI */
        rcc.regs.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());

        spi.cr1.write(|w| w.spe().clear_bit() ); // disable SPI
        spi.cr2.write(|w|
            unsafe {
                w
                    .frxth()
                    .set_bit()
                    .ds()
                    .bits(0b1111) // 16 bits
                    .ssoe()
                    .clear_bit()
            }
        );

        spi.cr1.write(|w|
            w
                .bidimode()
                .clear_bit()
                .rxonly()
                .clear_bit()
//                .br()
//                .bits(0b010) // 8 Mhz pclk / 1 Mhz bus speed
                .ssm()
                .set_bit()
                .ssi()
                .set_bit()
                .lsbfirst()
                .bit(false)
                .mstr().set_bit()
                .cpha()
                .bit(MODE_0.phase == Phase::CaptureOnSecondTransition)
                .cpol()
                .bit(MODE_0.polarity == Polarity::IdleHigh)
                .spe()
                .set_bit()
        );

        MCP4922 { spi, sck, mosi, cs }
    }

    pub fn set_voltage(&mut self, channel: Channel, voltage: f32) {
        let clamped_voltage: f32;
        if voltage < 0.0 {
            clamped_voltage = 0.0;
        } else if voltage > 3.3 {
            clamped_voltage = 3.3;
        } else {
            clamped_voltage = voltage;
        }

        let duty: u16 = ((clamped_voltage * (2_u16.pow(12) as f32 - 1.0)) / 3.3)  as u16;
        let header: u16;
        match channel {
            Channel::A => header = 0b0011,
            Channel::B => header = 0b1011
        }

        self.cs.set_low();
        block!(self.write_data_register(header << 12 | duty));
        self.cs.set_high();
    }

    fn write_data_register(&mut self, data: u16) -> nb::Result<(), hal::spi::Error> {
        let sr = self.spi.sr.read();

        if sr.ovr().bit_is_set() {
            self.spi.dr.read();
            self.spi.sr.read();
//            Err(nb::Error::Other(hal::spi::Error::Overrun))
            Err(nb::Error::WouldBlock)
        } else if sr.modf().bit_is_set() {
            asm::bkpt();
            Err(nb::Error::Other(hal::spi::Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            asm::bkpt();
            Err(nb::Error::Other(hal::spi::Error::Crc))
        } else if sr.txe().bit_is_set() {
            self.spi.dr.write(|w| unsafe { w.bits(data as u32) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
