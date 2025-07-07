use super::{BMX160, Error, defs};
use embedded_hal::i2c::{I2c, SevenBitAddress};
use micromath::vector::Vector3d;

impl<I, E> BMX160<I>
where
    I: I2c<SevenBitAddress, Error = E>,
{
    pub fn get_accel_conf(&mut self) -> Result<AccelConfig, Error<E>> {
        let mut data: [u8; 2] = [0; 2];

        // Read current accelerometer configuration (2 bytes)
        self.read_bytes(defs::BMI160_ACCEL_CONF_ADDR, &mut data)
            .map_err(Error::I2c)?;

        // Extract Output Data Rate (ODR) from data[0]
        let odr = match (data[0] & defs::BMI160_ACCEL_ODR_MASK).try_into() {
            Ok(odr) => odr,
            Err(_) => return Err(Error::InvalidMode),
        };

        // Extract Bandwidth (BW) from data[0] (bits 4-6)
        let bw = match ((data[0] & defs::BMI160_ACCEL_BW_MASK) >> 4).try_into() {
            Ok(bw) => bw,
            Err(_) => return Err(Error::InvalidMode),
        };

        let undersample = (data[0] & defs::BMI160_ACCEL_UNDERSAMPLING_MASK) >> 7 == 1;

        // Extract Accelerometer Range from data[1]
        let range = match (data[1] & defs::BMI160_ACCEL_RANGE_MASK).try_into() {
            Ok(range) => range,
            Err(_) => return Err(Error::InvalidMode),
        };

        // Create AccelConfig from extracted values
        Ok(AccelConfig {
            odr,
            range,
            bandwidth: bw,
            undersample,
        })
    }

    pub fn set_accel_conf(&mut self, conf: AccelConfig) -> Result<(), Error<E>> {
        let mut data: [u8; 2] = [0; 2];

        // Read current accelerometer configuration
        self.read_bytes(defs::BMI160_ACCEL_CONF_ADDR, &mut data)
            .map_err(Error::I2c)?;

        let odr_val = conf.odr as u8;
        let bw_val = conf.bandwidth as u8;
        let range_val = conf.range as u8;
        let undersample_val = conf.undersample as u8;

        // --- Set Output Data Rate (ODR) ---
        data[0] =
            (data[0] & !defs::BMI160_ACCEL_ODR_MASK) | (odr_val & defs::BMI160_ACCEL_ODR_MASK);

        // --- Set Bandwidth (BW) ---
        data[0] =
            (data[0] & !defs::BMI160_ACCEL_BW_MASK) | ((bw_val << 4) & defs::BMI160_ACCEL_BW_MASK);

        // --- Set undersample (US) ---
        data[0] = (data[0] & !defs::BMI160_ACCEL_UNDERSAMPLING_MASK)
            | ((undersample_val << 7) & defs::BMI160_ACCEL_BW_MASK);

        // Write updated ODR and BW (data[0]) to ACCEL_CONF_ADDR
        self.write_u8(defs::BMI160_ACCEL_CONF_ADDR, data[0])
            .map_err(Error::I2c)?;

        // --- Set Accelerometer Range ---
        data[1] = (data[1] & !defs::BMI160_ACCEL_RANGE_MASK)
            | (range_val & defs::BMI160_ACCEL_RANGE_MASK);

        // Write updated range (data[1]) to ACCEL_RANGE_ADDR
        self.write_u8(defs::BMI160_ACCEL_RANGE_ADDR, data[1])
            .map_err(Error::I2c)?;

        if conf.undersample {
            // Disable interrupt pre-filter data in when undersampling
            let pre_filter: [u8; 2] = [0; 2];
            self.write_bytes(defs::BMI160_INT_DATA_0_ADDR, &pre_filter)?;
        }

        Ok(())
    }

    /// Read raw accelerometer data
    pub fn get_raw_accel_data(&mut self) -> Result<Vector3d<i16>, Error<E>> {
        let mut data_array: [u8; 6] = [0; 6];

        // Read 9 bytes: 6 for accel data, 3 for sensor time
        self.read_bytes(defs::BMI160_ACCEL_DATA_ADDR, &mut data_array)
            .map_err(Error::I2c)?;

        // Parse accelerometer X, Y, Z data
        let x = i16::from_le_bytes([data_array[0], data_array[1]]);
        let y = i16::from_le_bytes([data_array[2], data_array[3]]);
        let z = i16::from_le_bytes([data_array[4], data_array[5]]);

        Ok(Vector3d { x, y, z })
    }

    pub fn get_accel_data(&mut self) -> Result<Vector3d<f32>, Error<E>> {
        let accel_data = self.get_raw_accel_data()?;
        let range = self.get_accel_conf()?.range.multiplier();

        Ok(Vector3d {
            x: accel_data.x as f32 * range,
            y: accel_data.y as f32 * range,
            z: accel_data.z as f32 * range,
        })
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelConfig {
    pub odr: AccelODR,
    pub range: AccelRange,
    /// Determines filter configuration (undersample = false) and averaging for undersampling mode (undersample = true)
    /// See datasheet 2.2.1.1.1
    pub bandwidth: AccelBandwidth,
    /// Typically used for low-power applications
    pub undersample: bool,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelODR {
    Hz0_78 = 0x01,
    Hz1_56 = 0x02,
    Hz3_12 = 0x03,
    Hz6_25 = 0x04,
    Hz12_5 = 0x05,
    Hz25 = 0x06,
    Hz50 = 0x07,
    Hz100 = 0x08,
    Hz200 = 0x09,
    Hz400 = 0x0A,
    Hz800 = 0x0B,
    Hz1600 = 0x0C,
}

impl TryFrom<u8> for AccelODR {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(AccelODR::Hz0_78),
            0x02 => Ok(AccelODR::Hz1_56),
            0x03 => Ok(AccelODR::Hz3_12),
            0x04 => Ok(AccelODR::Hz6_25),
            0x05 => Ok(AccelODR::Hz12_5),
            0x06 => Ok(AccelODR::Hz25),
            0x07 => Ok(AccelODR::Hz50),
            0x08 => Ok(AccelODR::Hz100),
            0x09 => Ok(AccelODR::Hz200),
            0x0A => Ok(AccelODR::Hz400),
            0x0B => Ok(AccelODR::Hz800),
            0x0C => Ok(AccelODR::Hz1600),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelRange {
    G2 = 0x03,
    G4 = 0x05,
    G8 = 0x08,
    G16 = 0x0C,
}

impl AccelRange {
    pub(crate) fn multiplier(self) -> f32 {
        match self {
            AccelRange::G2 => 1. / 16384.,
            AccelRange::G4 => 1. / 8192.,
            AccelRange::G8 => 1. / 4096.,
            AccelRange::G16 => 1. / 2048.,
        }
    }
}

impl TryFrom<u8> for AccelRange {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x03 => Ok(AccelRange::G2),
            0x05 => Ok(AccelRange::G4),
            0x08 => Ok(AccelRange::G8),
            0x0C => Ok(AccelRange::G16),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelBandwidth {
    OSR4 = 0x00,
    OSR2 = 0x01,
    Normal = 0x02,
}

impl TryFrom<u8> for AccelBandwidth {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(AccelBandwidth::OSR4),
            0x01 => Ok(AccelBandwidth::OSR2),
            0x02 => Ok(AccelBandwidth::Normal),
            _ => Err(()),
        }
    }
}
