use stm32f0xx_hal as hal;
use hal::gpio::{Alternate, AF0, gpioa::{PA5, PA6, PA7}};

pub trait SckPin<SPI> {}
pub trait MisoPin<SPI> {}
pub trait MosiPin<SPI> {}

impl SckPin<hal::stm32::SPI1> for PA5<Alternate<AF0>> {}
impl MisoPin<hal::stm32::SPI1> for PA6<Alternate<AF0>> {}
impl MosiPin<hal::stm32::SPI1> for PA7<Alternate<AF0>> {}