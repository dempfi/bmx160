use super::accel::AccelRange;
use super::{BMX160, Error, defs};
use embedded_hal::i2c::{I2c, SevenBitAddress};

impl<I, E> BMX160<I>
where
    I: I2c<SevenBitAddress, Error = E>,
{
    pub fn set_tap_interrupt_conf(&mut self, conf: TapIntConf) -> Result<(), Error<E>> {
        let mut data: [u8; 2] = [0; 2];

        // Read the current INT_TAP register values
        self.read_bytes(defs::BMI160_INT_TAP_ADDR, &mut data)
            .map_err(Error::I2c)?;

        // Configure INT_TAP[0] - Quiet, Shock, Duration
        data[0] = (data[0] & 0b0000_1111)               // Preserve lower 4 bits (tap duration)
      | ((conf.quiet as u8) << 7)               // Set quiet (Bit 7)
      | ((conf.shock as u8) << 6)               // Set shock (Bit 6)
      | ((conf.duration as u8) & 0b111); // Set tap duration (Bits 0-2)

        // Configure INT_TAP[1] - Threshold
        data[1] = conf.threshold as u8 & 0x0F; // Set threshold (Bits 0-3)

        self.write_bytes(defs::BMI160_INT_TAP_ADDR, &data)?;

        Ok(())
    }
}

/// Quiet duration for tap detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapQuiet {
    Ms30 = 0,
    Ms20 = 1,
}

/// Shock duration for tap detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapShock {
    Ms50 = 0,
    Ms75 = 1,
}

/// Tap duration for double-tap detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapDuration {
    Ms50 = 0b000,
    Ms100 = 0b001,
    Ms150 = 0b010,
    Ms200 = 0b011,
    Ms250 = 0b100,
    Ms375 = 0b101,
    Ms500 = 0b110,
    Ms700 = 0b111,
}

/// Represents the configurable tap threshold levels for the BMX160 sensor.
///
/// This threshold determines the minimum acceleration required to trigger a
/// single or double-tap interrupt. The value is applied as a multiplier to
/// a scaling factor that depends on the selected accelerometer range (±2g, ±4g, ±8g, ±16g).
///
/// ## Threshold Calculation:
/// The tap threshold is calculated using the following formula:
/// ```text
/// threshold (mg) = val(int_tap_th<3:0>) * scaling_factor
/// ```
/// Where:
/// - `int_tap_th<3:0>` is the 4-bit value representing the tap threshold (0 to 15).
/// - `scaling_factor` depends on the accelerometer range:
///   - ±2g  → 62.5 mg
///   - ±4g  → 125 mg
///   - ±8g  → 250 mg
///   - ±16g → 500 mg
///
/// ## Special Case (Level0):
/// If `TapThreshold::Level0` (0b0000) is selected, the multiplier is treated as **0.5**.
/// This allows for finer sensitivity at lower thresholds:
/// ```text
/// threshold (mg) = 0.5 * scaling_factor
/// ```
///
/// ## Usage Example:
/// ```rust
/// let threshold = TapThreshold::Level3;
/// let mg_value = threshold.to_mg(TapRange::G4); // Calculates mg for 4g range
/// println!("Threshold: {} mg", mg_value); // Output: 375 mg (for 4g range)
/// ```
///
/// This enum helps set the appropriate tap sensitivity, allowing the sensor to detect
/// lighter or stronger taps based on the configured value.
///
/// ## Applications:
/// - **Lower threshold values (Lvl0 - Lvl3)**: Detect light taps or soft touches.
/// - **Higher threshold values (Lvl10 - Lvl15)**: Detect stronger taps, reducing false positives.
///
/// By selecting the appropriate threshold, you can fine-tune tap detection for applications like:
/// - Wearable devices (double-tap to wake)
/// - Gesture-based controls
/// - Impact detection
///
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapThreshold {
    Lvl0 = 0b0000,
    Lvl1 = 0b0001,
    Lvl2 = 0b0010,
    Lvl3 = 0b0011,
    Lvl4 = 0b0100,
    Lvl5 = 0b0101,
    Lvl6 = 0b0110,
    Lvl7 = 0b0111,
    Lvl8 = 0b1000,
    Lvl9 = 0b1001,
    Lvl10 = 0b1010,
    Lvl11 = 0b1011,
    Lvl12 = 0b1100,
    Lvl13 = 0b1101,
    Lvl14 = 0b1110,
    Lvl15 = 0b1111,
}

impl TapThreshold {
    /// Calculates the tap threshold in mg based on the selected accelerometer range.
    ///
    /// This method applies the tap threshold multiplier to the appropriate scaling factor
    /// depending on the provided `TapRange`.
    ///
    /// - `range` – The selected accelerometer range (±2g, ±4g, ±8g, or ±16g).
    /// - Returns the calculated threshold in mg (milligrams).
    ///
    /// ## Example:
    /// ```rust
    /// let threshold = TapThreshold::Level5;
    /// let mg = threshold.to_mg(TapRange::G8);  // 1250 mg for 8g range
    /// println!("Threshold: {} mg", mg);
    /// ```
    pub fn to_mg(&self, range: AccelRange) -> f32 {
        let scaling_factor = match range {
            AccelRange::G2 => 62.5,
            AccelRange::G4 => 125.0,
            AccelRange::G8 => 250.0,
            AccelRange::G16 => 500.0,
        };

        // Apply 0.5 multiplier for Level0
        let multiplier = if *self == TapThreshold::Lvl0 {
            0.5
        } else {
            *self as u8 as f32
        };

        multiplier * scaling_factor
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapIntConf {
    pub quiet: TapQuiet,
    pub shock: TapShock,
    pub duration: TapDuration,
    pub threshold: TapThreshold,
}
