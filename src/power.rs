use super::{defs, Error, BMX160};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};

impl<I, E> BMX160<I>
where
  I: I2c<SevenBitAddress, Error = E>,
{
  pub fn get_power_mode(&mut self) -> Result<PowerMode, Error<E>> {
    let mut pmu_status: [u8; 1] = [0; 1];

    // Read PMU status from the device
    self
      .read_bytes(defs::BMI160_PMU_STATUS_ADDR, &mut pmu_status)
      .map_err(Error::I2c)?;

    let power_mode = pmu_status[0];

    // Extract power modes from PMU status
    let accel_power =
      (power_mode & defs::BMI160_ACCEL_POWER_MODE_MSK) >> defs::BMI160_ACCEL_POWER_MODE_POS;
    let gyro_power =
      (power_mode & defs::BMI160_GYRO_POWER_MODE_MSK) >> defs::BMI160_GYRO_POWER_MODE_POS;
    let mag_power =
      (power_mode & defs::BMI160_MAG_POWER_MODE_MSK) >> defs::BMI160_MAG_POWER_MODE_POS;

    // Convert to enums using TryFrom
    let accel_mode = AccelPowerMode::try_from(accel_power).map_err(|_| Error::InvalidMode)?;
    let gyro_mode = GyroPowerMode::try_from(gyro_power).map_err(|_| Error::InvalidMode)?;
    let mag_mode = MagPowerMode::try_from(mag_power).map_err(|_| Error::InvalidMode)?;

    // Return the power modes as a struct
    Ok(PowerMode { accel: accel_mode, gyro: gyro_mode, mag: mag_mode })
  }

  pub fn set_power_mode(
    &mut self,
    mode: PowerMode,
    delay: &mut dyn DelayNs,
  ) -> Result<(), Error<E>> {
    let accel_cmd = 0b0001_00_00 | (mode.accel as u8); // 0b0001 00nn
    let gyro_cmd = 0b0001_01_00 | (mode.gyro as u8); // 0b0001 01nn
    let mag_cmd = 0b0001_10_00 | (mode.mag as u8); // 0b0001 10nn

    let current = self.get_power_mode()?;

    if mode.accel != current.accel {
      // Write new accel power mode to the command register
      self
        .write_u8(defs::BMI160_COMMAND_REG_ADDR, accel_cmd)
        .map_err(Error::I2c)?;

      // Delay if transitioning from suspend mode
      if current.accel != AccelPowerMode::Normal {
        delay.delay_ms(defs::BMI160_ACCEL_DELAY_MS);
      }
    }

    if mode.gyro != current.gyro {
      self
        .write_u8(defs::BMI160_COMMAND_REG_ADDR, gyro_cmd)
        .map_err(Error::I2c)?;

      // Delay if transitioning from suspend mode
      if current.gyro != GyroPowerMode::Normal {
        delay.delay_ms(defs::BMI160_GYRO_DELAY_MS);
      }
    }

    if mode.mag != current.mag {
      self
        .write_u8(defs::BMI160_COMMAND_REG_ADDR, mag_cmd)
        .map_err(Error::I2c)?;

      // Magnetometer delay is often negligible, but add a small buffer if needed
      delay.delay_ms(5);
    }

    Ok(())
  }
}

#[derive(Debug, defmt::Format)]
pub struct PowerMode {
  pub accel: AccelPowerMode,
  pub gyro: GyroPowerMode,
  pub mag: MagPowerMode,
}

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum AccelPowerMode {
  Suspend = 0b00,
  Normal = 0b01,
  LowPower = 0b10,
}

impl TryFrom<u8> for AccelPowerMode {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0b00 => Ok(AccelPowerMode::Suspend),
      0b01 => Ok(AccelPowerMode::Normal),
      0b10 => Ok(AccelPowerMode::LowPower),
      _ => Err(()),
    }
  }
}

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum GyroPowerMode {
  Suspend = 0b00,
  Normal = 0b01,
  FastStartUp = 0b11,
}

impl TryFrom<u8> for GyroPowerMode {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0b00 => Ok(GyroPowerMode::Suspend),
      0b01 => Ok(GyroPowerMode::Normal),
      0b11 => Ok(GyroPowerMode::FastStartUp),
      _ => Err(()),
    }
  }
}

#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum MagPowerMode {
  Suspend = 0b00,
  Normal = 0b01,
  LowPower = 0b10,
}

impl TryFrom<u8> for MagPowerMode {
  type Error = ();

  fn try_from(value: u8) -> Result<Self, Self::Error> {
    match value {
      0b00 => Ok(MagPowerMode::Suspend),
      0b01 => Ok(MagPowerMode::Normal),
      0b10 => Ok(MagPowerMode::LowPower),
      _ => Err(()),
    }
  }
}
