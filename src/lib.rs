#![no_std]

use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

pub mod accel;
mod defs;
pub mod gyro;
pub mod interrupt;
pub mod motion;
pub mod power;
pub mod tap;

/// All possible errors in this crate
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    I2c(E),
    InvalidChipId(u8),
    InvalidMode,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BMX160<I> {
    i2c: I,
}

impl<I, E> BMX160<I>
where
    I: I2c<SevenBitAddress, Error = E>,
{
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    pub fn id(&mut self) -> Result<u8, Error<E>> {
        self.read_u8(defs::BMX160_CHIP_ID_ADDR).map_err(Error::I2c)
    }

    pub fn soft_reset(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        self.write_u8(defs::BMI160_COMMAND_REG_ADDR, defs::BMI160_SOFT_RESET_CMD)
            .map_err(Error::I2c)?;

        delay.delay_ms(defs::BMI160_SOFT_RESET_DELAY_MS as u32);

        Ok(())
    }

    pub fn init(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        let id = self.id()?;
        if id != defs::BMX160_CHIP_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.soft_reset(delay)?;

        Ok(())
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];

        match self
            .i2c
            .write_read(defs::BMX160_I2C_ADDR, &[reg], &mut byte)
        {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(defs::BMX160_I2C_ADDR, &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(defs::BMX160_I2C_ADDR, &[reg, value])?;

        Ok(())
    }

    fn write_bytes(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<E>> {
        for (i, byte) in data.iter().enumerate() {
            self.write_u8(reg + i as u8, *byte).map_err(Error::I2c)?;
        }
        Ok(())
    }
}
