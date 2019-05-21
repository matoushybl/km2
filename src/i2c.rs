//#![feature(never_type)]
//
//use core::ops::Deref;
//use core::marker::PhantomData;
//
//use embedded_hal::blocking::i2c::{Write, WriteRead, Read};
//
//use stm32f0xx_hal::{
//    gpio::*,
//    rcc::Rcc,
//    time::{KiloHertz, U32Ext},
//};
//
//use cortex_m::asm;
//
//extern crate nb;
//
//pub struct Master;
//pub struct Slave;
//
///// I2C abstraction
//pub struct I2c<I2C, SCLPIN, SDAPIN, MODE> {
//    i2c: I2C,
//    pins: (SCLPIN, SDAPIN),
//    _mode: PhantomData<MODE>
//}
//
//pub trait SclPin<I2C> {}
//pub trait SdaPin<I2C> {}
//
//macro_rules! i2c_pins {
//    ($($I2C:ident => {
//        scl => [$($scl:ty),+ $(,)*],
//        sda => [$($sda:ty),+ $(,)*],
//    })+) => {
//        $(
//            $(
//                impl SclPin<crate::stm32::$I2C> for $scl {}
//            )+
//            $(
//                impl SdaPin<crate::stm32::$I2C> for $sda {}
//            )+
//        )+
//    }
//}
//
//i2c_pins! {
//    I2C1 => {
//        scl => [gpioa::PA9<Alternate<AF4>>],
//        sda => [gpioa::PA10<Alternate<AF4>>],
//    }
//}
//
//#[derive(Debug)]
//pub enum Error {
//    OVERRUN,
//    ArbitrationLost,
//    BusError,
//    NACK,
//}
//
//macro_rules! i2c {
//    ($($I2C:ident: ($i2c:ident, $i2cXen:ident, $i2cXrst:ident, $apbenr:ident, $apbrstr:ident),)+) => {
//        $(
//            use crate::stm32::$I2C;
//            impl<SCLPIN, SDAPIN> I2c<$I2C, SCLPIN, SDAPIN, Master> {
//                pub fn $i2c(i2c: $I2C, pins: (SCLPIN, SDAPIN), speed: KiloHertz, rcc: &mut Rcc) -> Self
//                where
//                    SCLPIN: SclPin<$I2C>,
//                    SDAPIN: SdaPin<$I2C>,
//                {
//                    // Enable clock for I2C
//                    rcc.regs.$apbenr.modify(|_, w| w.$i2cXen().set_bit());
//
//                    // Reset I2C
//                    rcc.regs.$apbrstr.modify(|_, w| w.$i2cXrst().set_bit());
//                    rcc.regs.$apbrstr.modify(|_, w| w.$i2cXrst().clear_bit());
//                    I2c { i2c, pins, _mode: PhantomData }.i2c_init(speed)
//                }
//            }
//        )+
//    }
//}
//
//i2c! {
//    I2C1: (i2c1, i2c1en, i2c1rst, apb1enr, apb1rstr),
//}
//
//i2c! {
//    I2C2: (i2c2, i2c2en, i2c2rst, apb1enr, apb1rstr),
//}
//
//// It's s needed for the impls, but rustc doesn't recognize that
//#[allow(dead_code)]
//type I2cRegisterBlock = crate::stm32::i2c1::RegisterBlock;
//
//impl<I2C, SCLPIN, SDAPIN, MODE> I2c<I2C, SCLPIN, SDAPIN, MODE>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
//    fn i2c_init(self: Self, speed: KiloHertz) -> Self {
//        use core::cmp;
//
//        // Make sure the I2C unit is disabled so we can configure it
//        self.i2c.cr1.modify(|_, w| w.pe().clear_bit());
//
//        // Calculate settings for I2C speed modes
//        let presc;
//        let scldel;
//        let sdadel;
//        let sclh;
//        let scll;
//
//        // We're using HSI here which runs at a fixed 8MHz
//        const FREQ: u32 = 8_000_000;
//
//        // Normal I2C speeds use a different scaling than fast mode below
//        if speed <= 100_u32.khz() {
//            presc = 1;
//            scll = cmp::max((((FREQ >> presc) >> 1) / speed.0) - 1, 255) as u8;
//            sclh = scll - 4;
//            sdadel = 2;
//            scldel = 4;
//        } else {
//            presc = 0;
//            scll = cmp::max((((FREQ >> presc) >> 1) / speed.0) - 1, 255) as u8;
//            sclh = scll - 6;
//            sdadel = 1;
//            scldel = 3;
//        }
//
//        // Enable I2C signal generator, and configure I2C for 400KHz full speed
//        self.i2c.timingr.write(|w| {
//            w.presc()
//                .bits(presc as u8)
//                .scldel()
//                .bits(scldel)
//                .sdadel()
//                .bits(sdadel)
//                .sclh()
//                .bits(sclh)
//                .scll()
//                .bits(scll)
//        });
//
////        hprintln!("prescaler {} scldel {} sdadel {} scll {} sclh {}", presc, scldel, sdadel, scll, sclh).unwrap();
//
//        // Enable the I2C processing
//        self.i2c.cr1.write(|w|
//            w.pe()
//                .set_bit());
//
//        self
//    }
//
//    pub fn release(self) -> (I2C, (SCLPIN, SDAPIN)) {
//        (self.i2c, self.pins)
//    }
//
//    fn check_and_clear_error_flags(&self, isr: &crate::stm32::i2c1::isr::R) -> Result<(), Error> {
//        // If we received a NACK, then this is an error
//        if isr.nackf().bit_is_set() {
//            self.i2c
//                .icr
//                .write(|w| w.stopcf().set_bit().nackcf().set_bit());
//            asm::bkpt();
//            return Err(Error::NACK);
//        }
//
//        if isr.ovr().bit_is_set() {
//            self.i2c
//                .icr
//                .write(|w| w.stopcf().set_bit().ovrcf().set_bit());
//            asm::bkpt();
//            return Err(Error::OVERRUN);
//        }
//
//        if isr.arlo().bit_is_set() {
//            self.i2c
//                .icr
//                .write(|w| w.stopcf().set_bit().arlocf().set_bit());
//            asm::bkpt();
//            return Err(Error::ArbitrationLost);
//        }
//
//        if isr.berr().bit_is_set() {
//            self.i2c
//                .icr
//                .write(|w| w.stopcf().set_bit().berrcf().set_bit());
//            asm::bkpt();
//            return Err(Error::BusError);
//        }
//
//        Ok(())
//    }
//
//    fn send_byte(&self, byte: u8) -> Result<(), Error> {
//        // Wait until we're ready for sending
//        while {
//            let isr = self.i2c.isr.read();
//            self.check_and_clear_error_flags(&isr)?;
//            isr.txis().bit_is_clear()
//        } {}
//
//        // Push out a byte of data
//        self.i2c.txdr.write(|w| unsafe { w.bits(u32::from(byte)) });
//
//        self.check_and_clear_error_flags(&self.i2c.isr.read())?;
//        Ok(())
//    }
//
//    pub fn recv_byte(&self) -> Result<u8, Error> {
//        while {
//            let isr = self.i2c.isr.read();
//            self.check_and_clear_error_flags(&isr)?;
//            isr.rxne().bit_is_clear()
//        } {}
//
//        let value = self.i2c.rxdr.read().bits() as u8;
//        Ok(value)
//    }
//
//    pub fn into_slave(self) -> I2c<I2C, SCLPIN, SDAPIN, Slave> {
//        return I2c {i2c: self.i2c, pins: self.pins, _mode: PhantomData }
//    }
//}
//
//pub trait SlaveAddress {
//    fn set_own_slave_addr(&self, addr: u8);
//    fn wait_for_addressing(&self) -> nb::Result<(), Error>;
//}
//
//impl<I2C, SCLPIN, SDAPIN> SlaveAddress for I2c<I2C, SCLPIN, SDAPIN, Slave>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
////    type Error = Error;
//
//    fn set_own_slave_addr(&self, addr: u8) {
////        self.i2c.cr1.write(|w| {
////            w.addrie().set_bit();
////        });
//        self.i2c.oar1.write(|w| {
//            w.oa1en().clear_bit()
//        });
//        self.i2c.oar2.write(|w| {
//            w.oa2en().clear_bit()
//        });
//        self.i2c.oar1.modify(|r, w| {
//            w.oa1().bits(u16::from(addr) << 1);
//            w.oa1en().set_bit();
//            w.oa1mode().clear_bit()
//        });
//    }
//
//    fn wait_for_addressing(&self) -> nb::Result<(), Error> {
//        let isr = self.i2c.isr.read();
//        self.check_and_clear_error_flags(&isr)?;
//        if isr.addr().bit_is_clear() {
//            return Err(nb::Error::WouldBlock);
//        }
//
//        self.i2c.icr.write(|w| { w.addrcf().set_bit() });
//
//        Ok(())
//    }
//}
//
//pub trait SlaveRead {
//    type Error;
//
//    fn read(&mut self, buffer: &mut [u8]) -> nb::Result<(), Error>;
//}
//
//impl<I2C, SCLPIN, SDAPIN> SlaveRead for I2c<I2C, SCLPIN, SDAPIN, Slave>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
//    type Error = Error;
//
//    fn read(&mut self, buffer: &mut [u8]) -> nb::Result<(), Error> {
//        // Now read in all bytes
//        for c in buffer.iter_mut() {
//            *c = self.recv_byte()?;
//        }
//
//        // Check and clear flags if they somehow ended up set
//        self.check_and_clear_error_flags(&self.i2c.isr.read())?;
//
//        Ok(())
//    }
//}
//
//pub trait SlaveWrite {
//    type Error;
//
//    fn write(&mut self, data: &[u8]) -> nb::Result<(), Error>;
//}
//
//
//impl<I2C, SCLPIN, SDAPIN> SlaveWrite for I2c<I2C, SCLPIN, SDAPIN, Slave>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
//    type Error = Error;
//
//    fn write(&mut self, bytes: &[u8]) -> nb::Result<(), Error> {
//        // Send out all individual bytes
//        for c in bytes {
//            self.send_byte(*c)?;
//        }
//
//        Ok(())
//    }
//}
//
////pub trait SlaveReadWrite {
////    type Error;
////
////    fn read_write(&mut self, number_of_bytes: u8, (&[u8]) -> [u8])
////}
//
//
//impl<I2C, SCLPIN, SDAPIN> Read for I2c<I2C, SCLPIN, SDAPIN, Master>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
//    type Error = Error;
//
//    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
//        self.i2c.cr2.modify(|_, w| {
//            w.sadd()
//                .bits(u16::from(address) << 1)
//                .nbytes()
//                .bits(buffer.len() as u8)
//                .rd_wrn()
//                .set_bit()
//        });
//
//        // Send another START condition
//        self.i2c.cr2.modify(|_, w| w.start().set_bit());
//
//        // Send the autoend after setting the start to get a restart
//        self.i2c.cr2.modify(|_, w| w.autoend().set_bit());
//
//        // Now read in all bytes
//        for c in buffer.iter_mut() {
//            *c = self.recv_byte()?;
//        }
//
//        // Check and clear flags if they somehow ended up set
//        self.check_and_clear_error_flags(&self.i2c.isr.read())?;
//
//        Ok(())
//    }
//}
//
//impl<I2C, SCLPIN, SDAPIN> WriteRead for I2c<I2C, SCLPIN, SDAPIN, Master>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
//    type Error = Error;
//
//    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
//        // Set up current slave address for writing and disable autoending
//        self.i2c.cr2.modify(|_, w| {
//            w.sadd()
//                .bits(u16::from(addr) << 1)
//                .nbytes()
//                .bits(bytes.len() as u8)
//                .rd_wrn()
//                .clear_bit()
//                .autoend()
//                .clear_bit()
//        });
//
//        // Send a START condition
//        self.i2c.cr2.modify(|_, w| w.start().set_bit());
//
//        // Wait until the transmit buffer is empty and there hasn't been any error condition
//        while {
//            let isr = self.i2c.isr.read();
//            self.check_and_clear_error_flags(&isr)?;
//            isr.txis().bit_is_clear() && isr.tc().bit_is_clear()
//        } {}
//
//        // Send out all individual bytes
//        for c in bytes {
//            self.send_byte(*c)?;
//        }
//
//        // Wait until data was sent
//        while {
//            let isr = self.i2c.isr.read();
//            self.check_and_clear_error_flags(&isr)?;
//            isr.tc().bit_is_clear()
//        } {}
//
//        // Set up current address for reading
//        self.i2c.cr2.modify(|_, w| {
//            w.sadd()
//                .bits(u16::from(addr) << 1)
//                .nbytes()
//                .bits(buffer.len() as u8)
//                .rd_wrn()
//                .set_bit()
//        });
//
//        // Send another START condition
//        self.i2c.cr2.modify(|_, w| w.start().set_bit());
//
//        // Send the autoend after setting the start to get a restart
//        self.i2c.cr2.modify(|_, w| w.autoend().set_bit());
//
//        // Now read in all bytes
//        for c in buffer.iter_mut() {
//            *c = self.recv_byte()?;
//        }
//
//        // Check and clear flags if they somehow ended up set
//        self.check_and_clear_error_flags(&self.i2c.isr.read())?;
//
//        Ok(())
//    }
//}
//
//impl<I2C, SCLPIN, SDAPIN> Write for I2c<I2C, SCLPIN, SDAPIN, Master>
//    where
//        I2C: Deref<Target = I2cRegisterBlock>,
//{
//    type Error = Error;
//
//    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
//        // Set up current slave address for writing and enable autoending
//        self.i2c.cr2.modify(|_, w| {
//            w.sadd()
//                .bits(u16::from(addr) << 1)
//                .nbytes()
//                .bits(bytes.len() as u8)
//                .rd_wrn()
//                .clear_bit()
//                .autoend()
//                .set_bit()
//        });
//
//        // Send a START condition
//        self.i2c.cr2.modify(|_, w| w.start().set_bit());
//
//        // Send out all individual bytes
//        for c in bytes {
//            self.send_byte(*c)?;
//        }
//
//        // Check and clear flags if they somehow ended up set
//        self.check_and_clear_error_flags(&self.i2c.isr.read())?;
//
//        Ok(())
//    }
//}
