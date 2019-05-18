use nb;
pub use embedded_hal::spi::{Mode, Phase, Polarity};

use stm32f0xx_hal as hal;

use hal::stm32::SPI2;
use hal::gpio::{Alternate, AF0, gpioa::{PA5, PA6, PA7}};
use hal::rcc::{Clocks, Rcc};
use hal::time::Hertz;

use core::marker::PhantomData;
use core::ops::Deref;


use cortex_m::asm;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

// we want to be able to have three possible combinations - R, W, RW

/*
    let sck = PAX;
    let spi = SPI(periph.SPI1, sck) // SPI<SPI1, SCK, MOSI, MISO, NonDirectional>
        .into_bidirectional(mosi, miso) // BDSPI<SPI1, SCK, MOSI, MISO, BiDirectional>
        or
        .into_write_only(mosi)
        .into_read_only(miso)
*/

pub struct SPI<SPI, SCK, MISO, MOSI, MODE> {
    spi: SPI,
    sck: SCK,
    miso: Option<MISO>,
    mosi: Option<MOSI>,
    frame_size: u8,
    _mode: PhantomData<MODE>
}

#[allow(dead_code)]
type SpiRegisterBlock = hal::stm32::spi1::RegisterBlock;

impl<SpiPeriph, SCK, MISO, MOSI, MODE> SPI<SpiPeriph, SCK, MISO, MOSI, MODE>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock>
{
    fn read_data_register(&mut self) -> nb::Result<u16, Error> {
        let sr = self.spi.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bit_is_set() {
            Ok((self.spi.dr.read().bits() & 2^16) as u16)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write_data_register(&mut self, data: u16) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        if sr.ovr().bit_is_set() {
            asm::bkpt();
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            asm::bkpt();
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            asm::bkpt();
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bit_is_set() {
            self.spi.dr.write(|w| unsafe { w.bits(data as u32) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

pub struct NonDirectional;
pub struct SimplexRead;
pub struct SimplexWrite;
pub struct FullDuplex;
// TODO add bidirectional mode

pub trait SckPin<SPI> {}
pub trait MisoPin<SPI> {}
pub trait MosiPin<SPI> {}

impl SckPin<hal::stm32::SPI1> for PA5<Alternate<AF0>> {}
impl MisoPin<hal::stm32::SPI1> for PA6<Alternate<AF0>> {}
impl MosiPin<hal::stm32::SPI1> for PA7<Alternate<AF0>> {}

impl<SpiPeriph, SCK, MISO, MOSI, MODE> SPI<SpiPeriph, SCK, MISO, MOSI, MODE>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock> {
    pub fn spi1<F>(spi: SpiPeriph,
                sck: SCK,
                mode: Mode,
                speed: F,
                frame_size: u8,
                lsb_first: bool,
                rcc: &mut Rcc, interface_mode: MODE) -> SPI<SpiPeriph, SCK, MISO, MOSI, NonDirectional>
        where
            SCK: SckPin<SpiPeriph>,
            MISO: MisoPin<SpiPeriph>,
            MOSI: MosiPin<SpiPeriph>,
            F: Into<Hertz> {

        rcc.regs.apb2enr.modify(|_, w| w.spi1en().set_bit());

        /* Reset SPI */
        rcc.regs.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());

        SPI {
            spi,
            sck,
            miso: None,
            mosi: None,
            frame_size,
            _mode: PhantomData,
        }.init(mode, speed, lsb_first, rcc.clocks)
    }
}

// TODO implement slave

impl<SpiPeriph, SCK, MISO, MOSI> SPI<SpiPeriph, SCK, MISO, MOSI, NonDirectional>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock>,
{
    fn init<F>(self: Self, mode: Mode, speed: F, lsb_first: bool, clocks: Clocks) -> Self where F: Into<Hertz> {
        self.spi.cr1.write(|w| w.spe().clear_bit() ); // disable SPI

        self.spi.cr2.write(|w|
            unsafe {
                w
                    .frxth()
                    .bit(self.frame_size > 8)
                    .ds()
                    .bits(self.frame_size - 1)
                    .ssoe()
                    .clear_bit()
            }
        );

        // baud rate
        let br = match clocks.pclk().0 / speed.into().0 {
            0 => unreachable!(),
            1...2 => 0b000,
            3...5 => 0b001,
            6...11 => 0b010,
            12...23 => 0b011,
            24...47 => 0b100,
            48...95 => 0b101,
            96...191 => 0b110,
            _ => 0b111,
        };

        self.spi.cr1.write(|w|
            w
                .bidimode()
                .clear_bit()
                .rxonly()
                .clear_bit()
                // SSM and SSI bits should be fully configurable and the NSS pin should also be captured
                .ssm() // in our case NSS is disconnected, true is set for better compatibility, this should be handled in a better way
                .set_bit()
                .ssi()
                .set_bit()
                .lsbfirst()
                .bit(lsb_first)
                .mstr().set_bit()
                .cpha()
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
                .cpol()
                .bit(mode.polarity == Polarity::IdleHigh)
                .spe()
                .set_bit()
        );

        self
    }

    pub fn into_write_only(self, mosi: MOSI) -> SPI<SpiPeriph, SCK, MISO, MOSI, SimplexWrite> {
        SPI {
            spi: self.spi,
            sck: self.sck,
            miso: self.miso,
            mosi: Some(mosi),
            _mode: PhantomData,
            frame_size: self.frame_size
        }
    }

    pub fn into_read_only(self, miso: MISO) -> SPI<SpiPeriph, SCK, MISO, MOSI, SimplexRead> {
        self.spi.cr1.write(|w| w.spe().clear_bit() ); // disable SPI
        self.spi.cr1.write(|w| w.rxonly().set_bit() );
        self.spi.cr1.write(|w| w.spe().set_bit() ); // enable SPI

        SPI {
            spi: self.spi,
            sck: self.sck,
            miso: Some(miso),
            mosi: self.mosi,
            _mode: PhantomData,
            frame_size: self.frame_size
        }
    }

    pub fn into_full_duplex(self, miso: MISO, mosi: MOSI) -> SPI<SpiPeriph, SCK, MISO, MOSI, FullDuplex> {
        self.spi.cr1.write(|w| w.spe().clear_bit() ); // disable SPI

        self.spi.cr1.write(|w| w.spe().set_bit() ); // enable SPI

        SPI {
            spi: self.spi,
            sck: self.sck,
            miso: Some(miso),
            mosi: Some(mosi),
            _mode: PhantomData,
            frame_size: self.frame_size
        }
    }
}

pub trait Read {
    type Error;

    fn read(&mut self) -> nb::Result<u16, Self::Error>;
}

pub trait Send {
    type Error;

    fn send(&mut self, word: u16) -> nb::Result<(), Self::Error>;
}

pub trait FullDuplexCommunication: Read + Send {

}

impl<SpiPeriph, SCK, MISO, MOSI> Read
for SPI<SpiPeriph, SCK, MISO, MOSI, FullDuplex>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u16, Error> {
        self.read_data_register()
    }
}

impl<SpiPeriph, SCK, MISO, MOSI> Send
for SPI<SpiPeriph, SCK, MISO, MOSI, FullDuplex>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn send(&mut self, word: u16) -> nb::Result<(), Error> {
        self.write_data_register(word)
    }
}

impl<SpiPeriph, SCK, MISO, MOSI> FullDuplexCommunication
for SPI<SpiPeriph, SCK, MISO, MOSI, FullDuplex>
    where SpiPeriph: Deref<Target = SpiRegisterBlock> { }

impl<SpiPeriph, SCK, MISO, MOSI> Read
for SPI<SpiPeriph, SCK, MISO, MOSI, SimplexRead>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u16, Error> {
        self.read_data_register()
    }
}

impl<SpiPeriph, SCK, MISO, MOSI> Send
for SPI<SpiPeriph, SCK, MISO, MOSI, SimplexWrite>
    where
        SpiPeriph: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn send(&mut self, word: u16) -> nb::Result<(), Error> {
        self.write_data_register(word)
    }
}