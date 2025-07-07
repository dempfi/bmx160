use super::{defs, Error, BMX160};
use embedded_hal::i2c::{I2c, SevenBitAddress};

impl<I, E> BMX160<I>
where
  I: I2c<SevenBitAddress, Error = E>,
{
  pub fn set_int_pin_conf(
    &mut self,
    channel: IntChannel,
    config: IntPinConfig,
  ) -> Result<(), Error<E>> {
    let mut data: [u8; 2] = [0; 2]; // [INT_OUT_CTRL, INT_LATCH]

    self
      .read_bytes(defs::BMI160_INT_OUT_CTRL_ADDR, &mut data)
      .map_err(Error::I2c)?;

    // Apply configuration for INT1 (Channel1 or Both)
    if channel == IntChannel::Int1 || channel == IntChannel::Both {
      data[0] = (data[0] & !defs::BMI160_INT1_OUTPUT_EN_MASK) | ((config.output_enable as u8) << 3);
      data[0] = (data[0] & !defs::BMI160_INT1_OUTPUT_MODE_MASK) | ((config.mode as u8) << 2);
      data[0] =
        (data[0] & !defs::BMI160_INT1_OUTPUT_TYPE_MASK) | ((config.trigger_level as u8) << 1);
      data[0] = (data[0] & !defs::BMI160_INT1_EDGE_CTRL_MASK) | (config.trigger_type as u8);
    }

    // Apply configuration for INT2 (Channel2 or Both)
    if channel == IntChannel::Int2 || channel == IntChannel::Both {
      data[0] = (data[0] & !defs::BMI160_INT2_OUTPUT_EN_MASK) | ((config.output_enable as u8) << 7);
      data[0] = (data[0] & !defs::BMI160_INT2_OUTPUT_MODE_MASK) | ((config.mode as u8) << 6);
      data[0] =
        (data[0] & !defs::BMI160_INT2_OUTPUT_TYPE_MASK) | ((config.trigger_level as u8) << 5);
      data[0] = (data[0] & !defs::BMI160_INT2_EDGE_CTRL_MASK) | ((config.trigger_type as u8) << 4);
    }

    // Apply input enable and latch mode configuration in INT_LATCH
    if channel == IntChannel::Int1 || channel == IntChannel::Both {
      data[1] = (data[1] & !defs::BMI160_INT1_INPUT_EN_MASK) | ((config.input_enable as u8) << 4);
    }

    if channel == IntChannel::Int2 || channel == IntChannel::Both {
      data[1] = (data[1] & !defs::BMI160_INT2_INPUT_EN_MASK) | ((config.input_enable as u8) << 5);
    }

    // Latch mode (lower 4 bits of INT_LATCH)
    data[1] = (data[1] & !defs::BMI160_INT_LATCH_MASK) | (config.latch_mode as u8);

    // Write back to both INT_OUT_CTRL and INT_LATCH registers
    self.write_bytes(defs::BMI160_INT_OUT_CTRL_ADDR, &data)?;

    Ok(())
  }

  pub fn enable_int(&mut self, channel: IntChannel, int_type: IntType) -> Result<(), Error<E>> {
    let mut int_en: [u8; 3] = [0; 3];
    let mut int_map: [u8; 3] = [0; 3];

    // Read the current INT_EN (0x50 - 0x52) and INT_MAP (0x55 - 0x57) register values
    self
      .read_bytes(defs::BMI160_INT_EN_ADDR, &mut int_en)
      .map_err(Error::I2c)?;
    self
      .read_bytes(defs::BMI160_INT_MAP_ADDR, &mut int_map)
      .map_err(Error::I2c)?;

    // Enable the specific interrupt in INT_EN registers
    match int_type {
      IntType::SingleTap => int_en[0] |= 1 << 5,
      IntType::DoubleTap => int_en[0] |= 1 << 4,
      IntType::AnyMotion => {
        int_en[0] |= 1 << 0; // X
        int_en[0] |= 1 << 1; // Y
        int_en[0] |= 1 << 2; // Z
      }
      IntType::NoMotion => {
        int_en[2] |= 1 << 0; // X
        int_en[2] |= 1 << 1; // Y
        int_en[2] |= 1 << 2; // Z
      }
      IntType::HighG => {
        int_en[1] |= 1 << 0; // X
        int_en[1] |= 1 << 1; // Y
        int_en[1] |= 1 << 2; // Z
      }
      IntType::LowG => int_en[1] |= 1 << 3,
      IntType::DataReady => int_en[1] |= 1 << 4,
      IntType::FifoWatermark => int_en[1] |= 1 << 6,
      IntType::FifoFull => int_en[1] |= 1 << 5,
      IntType::Flat => int_en[0] |= 1 << 7,
      IntType::Orientation => int_en[0] |= 1 << 6,
    }

    // Map the enabled interrupt to INT1 or INT2 pins using INT_MAP registers
    match (channel, int_type) {
      // INT1 Mappings
      (IntChannel::Int1, IntType::SingleTap) => int_map[0] |= 1 << 5,
      (IntChannel::Int1, IntType::DoubleTap) => int_map[0] |= 1 << 4,
      (IntChannel::Int1, IntType::AnyMotion) => int_map[0] |= 1 << 2,
      (IntChannel::Int1, IntType::NoMotion) => int_map[0] |= 1 << 3,
      (IntChannel::Int1, IntType::HighG) => int_map[0] |= 1 << 1,
      (IntChannel::Int1, IntType::LowG) => int_map[0] |= 1 << 0,
      (IntChannel::Int1, IntType::DataReady) => int_map[1] |= 1 << 7,
      (IntChannel::Int1, IntType::FifoWatermark) => int_map[1] |= 1 << 6,
      (IntChannel::Int1, IntType::FifoFull) => int_map[1] |= 1 << 5,
      (IntChannel::Int1, IntType::Flat) => int_map[0] |= 1 << 7,
      (IntChannel::Int1, IntType::Orientation) => int_map[0] |= 1 << 6,

      // INT2 Mappings
      (IntChannel::Int2, IntType::SingleTap) => int_map[2] |= 1 << 5,
      (IntChannel::Int2, IntType::DoubleTap) => int_map[2] |= 1 << 4,
      (IntChannel::Int2, IntType::AnyMotion) => int_map[2] |= 1 << 2,
      (IntChannel::Int2, IntType::NoMotion) => int_map[2] |= 1 << 3,
      (IntChannel::Int2, IntType::HighG) => int_map[2] |= 1 << 1,
      (IntChannel::Int2, IntType::LowG) => int_map[2] |= 1 << 0,
      (IntChannel::Int2, IntType::DataReady) => int_map[1] |= 1 << 3,
      (IntChannel::Int2, IntType::FifoWatermark) => int_map[1] |= 1 << 2,
      (IntChannel::Int2, IntType::FifoFull) => int_map[1] |= 1 << 1,
      (IntChannel::Int2, IntType::Flat) => int_map[2] |= 1 << 7,
      (IntChannel::Int2, IntType::Orientation) => int_map[2] |= 1 << 6,

      // Handle Both Channels
      (IntChannel::Both, _) => {
        self.enable_int(IntChannel::Int1, int_type)?;
        self.enable_int(IntChannel::Int2, int_type)?;
        return Ok(());
      }
    }

    // Write the updated values to the INT_EN and INT_MAP registers
    self.write_bytes(defs::BMI160_INT_EN_ADDR, &int_en)?;
    self.write_bytes(defs::BMI160_INT_MAP_ADDR, &int_map)?;

    Ok(())
  }

  pub fn get_int_status(&mut self) -> Result<InterruptStatus, Error<E>> {
    let mut int_status: [u8; 4] = [0; 4];
    self
      .read_bytes(defs::BMI160_INT_STATUS_ADDR, &mut int_status)
      .map_err(Error::I2c)?;

    Ok(InterruptStatus {
      tap: self.decode_tap_status(int_status[2], int_status[0]),
      motion: self.decode_motion_status(int_status[2], int_status[1], int_status[0]),
      g_force: self.decode_g_force_status(int_status[3], int_status[1]),
      orientation: self.decode_orientation_status(int_status[3], int_status[0]),
      fifo: self.decode_fifo_status(int_status[1]),
      step: (int_status[0] & (1 << 0)) != 0,
      data: (int_status[1] & (1 << 4)) != 0,
    })
  }

  pub fn get_tap_int_status(&mut self) -> Result<Option<TapStatus>, Error<E>> {
    let mut int_status: [u8; 3] = [0; 3];
    self
      .read_bytes(defs::BMI160_INT_STATUS_ADDR, &mut int_status)
      .map_err(Error::I2c)?;

    Ok(self.decode_tap_status(int_status[2], int_status[0]))
  }

  fn decode_tap_status(&self, status2: u8, status0: u8) -> Option<TapStatus> {
    let active = status0 & ((1 << 5) | (1 << 4));
    if active == 0 {
      return None;
    }

    let direction = AxisDirection {
      axis: IntAxis::new(
        (status2 & (1 << 4)) != 0,
        (status2 & (1 << 5)) != 0,
        (status2 & (1 << 6)) != 0,
      ),
      sign: IntSign::new((status2 & (1 << 7)) != 0),
    };

    if active & (1 << 5) != 0 {
      Some(TapStatus::Single(direction))
    } else if active & (1 << 4) != 0 {
      Some(TapStatus::Double(direction))
    } else {
      None
    }
  }

  /// Decode motion-related interrupts: any-motion, no-motion, and significant motion.
  fn decode_motion_status(&self, status2: u8, status1: u8, status0: u8) -> Option<MotionStatus> {
    if status0 & (1 << 1) != 0 {
      Some(MotionStatus::Significant)
    } else if status1 & (1 << 7) != 0 {
      Some(MotionStatus::No)
    } else if status0 & (1 << 2) != 0 {
      let direction = AxisDirection {
        axis: IntAxis::new(
          (status2 & (1 << 0)) != 0,
          (status2 & (1 << 1)) != 0,
          (status2 & (1 << 2)) != 0,
        ),
        sign: IntSign::new((status2 & (1 << 3)) != 0),
      };

      Some(MotionStatus::Any(direction))
    } else {
      None
    }
  }

  fn decode_g_force_status(&self, status3: u8, status1: u8) -> Option<GForceStatus> {
    if status1 & (1 << 2) != 0 {
      let direction = AxisDirection {
        axis: IntAxis::new(
          (status3 & (1 << 0)) != 0,
          (status3 & (1 << 1)) != 0,
          (status3 & (1 << 2)) != 0,
        ),
        sign: IntSign::new((status3 & (1 << 3)) != 0),
      };

      Some(GForceStatus::High(direction))
    } else if status1 & (1 << 3) != 0 {
      Some(GForceStatus::Low)
    } else {
      None
    }
  }

  /// Decode orientation-related interrupts (flat and orient).
  fn decode_orientation_status(&self, status3: u8, status0: u8) -> Option<OrientationStatus> {
    if status0 & (1 << 7) != 0 {
      if (status3 & (1 << 6)) != 0 {
        Some(OrientationStatus::FaceDown)
      } else {
        Some(OrientationStatus::FaceUp)
      }
    } else if status0 & (1 << 6) != 0 {
      match (status3 >> 4) & 0b11 {
        0b00 => Some(OrientationStatus::PortraitUpright),
        0b01 => Some(OrientationStatus::PortraitUpsideDown),
        0b10 => Some(OrientationStatus::LandscapeLeft),
        0b11 => Some(OrientationStatus::LandscapeRight),
        _ => unreachable!(),
      }
    } else {
      None
    }
  }

  /// Decode FIFO-related interrupts
  fn decode_fifo_status(&self, status1: u8) -> Option<FifoStatus> {
    if (status1 & (1 << 6)) != 0 {
      Some(FifoStatus::Watermark)
    } else if (status1 & (1 << 5)) != 0 {
      Some(FifoStatus::Full)
    } else {
      None
    }
  }
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum IntChannel {
  Int1,
  Int2,
  Both,
}

/// Represents the possible latching modes for interrupts,
/// mapped to the INT_LATCH (0x54) register.
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum LatchMode {
  /// Non-latched mode (interrupt clears automatically).
  NonLatched = 0b0000,
  Us312_5 = 0b0001,
  Us625 = 0b0010,
  Ms1_25 = 0b0011,
  Ms2_5 = 0b0100,
  Ms5 = 0b0101,
  Ms10 = 0b0110,
  Ms20 = 0b0111,
  Ms40 = 0b1000,
  Ms80 = 0b1001,
  Ms160 = 0b1010,
  Ms320 = 0b1011,
  Ms640 = 0b1100,
  S1_28 = 0b1101,
  S2_56 = 0b1110,
  /// Fully latched mode (cleared manually).
  Latched = 0b1111,
}

/// Configures the pin output mode (push-pull or open-drain).
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum PinMode {
  /// Push-pull mode (default).
  PushPull = 0,
  /// Open-drain mode.
  OpenDrain = 1,
}

/// Represents the active level for interrupt pins.
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum TriggerLevel {
  /// Active low level (0).
  ActiveLow = 0,
  /// Active high level (1).
  ActiveHigh = 1,
}

/// Defines the trigger behavior for interrupt pins.
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum TriggerType {
  /// Level-triggered (default).
  Level = 0,
  /// Edge-triggered.
  Edge = 1,
}

/// Represents the configuration for interrupt pins and latch modes.
/// This struct combines settings for the INT_OUT_CTRL and INT_LATCH registers.
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct IntPinConfig {
  /// Enables or disables output for the interrupt pin.
  pub output_enable: bool,
  /// Defines whether the pin operates in push-pull or open-drain mode.
  pub mode: PinMode,
  /// Sets the active level for the interrupt pin (active high or low).
  pub trigger_level: TriggerLevel,
  /// Determines if the interrupt is edge-triggered or level-triggered.
  pub trigger_type: TriggerType,
  /// Sets the latching behavior for the interrupt (latched or temporary).
  pub latch_mode: LatchMode,
  /// Enables input on the pin.
  pub input_enable: bool,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum IntType {
  SingleTap,
  DoubleTap,
  AnyMotion,
  NoMotion,
  HighG,
  LowG,
  DataReady,
  FifoWatermark,
  FifoFull,
  Flat,
  Orientation,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum IntAxis {
  X,
  Y,
  Z,
}

impl IntAxis {
  fn new(x: bool, y: bool, z: bool) -> Self {
    match (x, y, z) {
      (true, false, false) => IntAxis::X,
      (false, true, false) => IntAxis::Y,
      (false, false, true) => IntAxis::Z,
      _ => panic!("Invalid axis orientation"),
    }
  }
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum IntSign {
  Negative,
  Positive,
}

impl IntSign {
  fn new(sign: bool) -> Self {
    if sign {
      IntSign::Negative
    } else {
      IntSign::Positive
    }
  }
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct AxisDirection {
  pub axis: IntAxis,
  pub sign: IntSign,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum OrientationStatus {
  PortraitUpright,
  PortraitUpsideDown,
  LandscapeLeft,
  LandscapeRight,
  FaceUp,
  FaceDown,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum FifoStatus {
  Watermark,
  Full,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum TapStatus {
  Single(AxisDirection),
  Double(AxisDirection),
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum MotionStatus {
  Significant,
  Any(AxisDirection),
  No,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum GForceStatus {
  High(AxisDirection),
  Low,
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct InterruptStatus {
  pub tap: Option<TapStatus>,
  pub orientation: Option<OrientationStatus>,
  pub motion: Option<MotionStatus>,
  pub g_force: Option<GForceStatus>,
  pub fifo: Option<FifoStatus>,
  pub step: bool,
  pub data: bool,
}
