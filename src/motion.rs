use super::{defs, Error, BMX160};
use embedded_hal::i2c::{I2c, SevenBitAddress};

impl<I, E> BMX160<I>
where
  I: I2c<SevenBitAddress, Error = E>,
{
  pub fn set_any_motion_interrupt_conf(&mut self, conf: AnyMotionIntConf) -> Result<(), Error<E>> {
    let mut data: [u8; 2] = [0; 2];

    // Read the current INT_TAP register values
    self
      .read_bytes(defs::BMI160_INT_MOTION_ADDR, &mut data)
      .map_err(Error::I2c)?;

    data[0] = (data[0] & 0b0000_0011) | (conf.duration as u8);
    data[1] = conf.threshold;

    self.write_bytes(defs::BMI160_INT_MOTION_ADDR, &data)?;

    Ok(())
  }

  /// TODO: Move to config struct
  pub fn set_motion_interrupts_to_use_filtered_data(
    &mut self,
    use_filter: bool,
  ) -> Result<(), Error<E>> {
    let mut data: [u8; 2] = [0; 2];

    // Read the current INT_TAP register values
    self
      .read_bytes(defs::BMI160_INT_MOTION_ADDR, &mut data)
      .map_err(Error::I2c)?;

    data[0] = (data[0] & 0b1111_1101) | ((use_filter as u8) << 1);

    self.write_bytes(defs::BMI160_INT_MOTION_ADDR, &data)?;

    Ok(())
  }

  pub fn set_significant_motion_interrupt(
    &mut self,
    conf: SignificantMotionIntConf,
  ) -> Result<(), Error<E>> {
    let sign_motion_select_bits = 0b0000_0010;
    let skip_bits = conf.skip as u8;
    let proof_bits = conf.proof as u8;
    let data = (skip_bits << 6) | (proof_bits << 4) | sign_motion_select_bits;

    self
      .write_u8(defs::BMI160_INT_MOTION_ADDR + 0x3, data)
      .map_err(Error::I2c)?;

    Ok(())
  }
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum AnyMotionDuration {
  One = 0,
  Two = 1,
  Three = 2,
  Four = 3,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct AnyMotionIntConf {
  pub duration: AnyMotionDuration,
  pub threshold: u8,
}

impl AnyMotionIntConf {
  pub fn new(duration: AnyMotionDuration, threshold: u8) -> Self {
    Self { duration, threshold }
  }
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum SignificantMotionSkip {
  S1_5 = 0,
  S3 = 1,
  S6 = 2,
  S12 = 3,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum SignificantMotionProof {
  S0_25 = 0,
  S0_5 = 1,
  S1 = 2,
  S2 = 3,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct SignificantMotionIntConf {
  pub skip: SignificantMotionSkip,
  pub proof: SignificantMotionProof,
}

impl SignificantMotionIntConf {
  pub fn new(skip: SignificantMotionSkip, proof: SignificantMotionProof) -> Self {
    Self { skip, proof }
  }
}
