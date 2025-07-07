use super::{BMX160, Error, defs};
use embedded_hal::i2c::{I2c, SevenBitAddress};
use micromath::vector::Vector3d;

impl<I, E> BMX160<I>
where
    I: I2c<SevenBitAddress, Error = E>,
{
    pub fn get_gyro_conf(&mut self) -> Result<GyroConfig, Error<E>> {
        let mut data: [u8; 2] = [0; 2];

        // Read current gyroscope configuration (2 bytes)
        self.read_bytes(defs::BMI160_GYRO_CONF_ADDR, &mut data)
            .map_err(Error::I2c)?;

        // Extract Gyro Output Data Rate (ODR) from data[0]
        let odr = GyroODR::try_from(data[0] & defs::BMI160_GYRO_ODR_MASK)
            .map_err(|_| Error::InvalidMode)?;

        // Extract Bandwidth (BW) from data[0] (bits 4-6)
        let bw = GyroBandwidth::try_from((data[0] & defs::BMI160_GYRO_BW_MASK) >> 4)
            .map_err(|_| Error::InvalidMode)?;

        // Extract Gyro Range from data[1]
        let range = GyroRange::try_from(data[1] & defs::BMI160_GYRO_RANGE_MASK)
            .map_err(|_| Error::InvalidMode)?;

        // Create GyroConfig from extracted values
        Ok(GyroConfig { odr, bw, range })
    }

    pub fn set_gyro_conf(&mut self, conf: GyroConfig) -> Result<(), Error<E>> {
        let mut data: [u8; 2] = [0; 2];

        // Read current gyroscope configuration
        self.read_bytes(defs::BMI160_GYRO_CONF_ADDR, &mut data)
            .map_err(Error::I2c)?;

        let odr_val = conf.odr as u8;
        let bw_val = conf.bw as u8;
        let range_val = conf.range as u8;

        // --- Set Gyro Output Data Rate (ODR) ---
        data[0] = (data[0] & !defs::BMI160_GYRO_ODR_MASK) | (odr_val & defs::BMI160_GYRO_ODR_MASK);

        // --- Set Gyro Bandwidth (BW) ---
        data[0] =
            (data[0] & !defs::BMI160_GYRO_BW_MASK) | ((bw_val << 4) & defs::BMI160_GYRO_BW_MASK);

        // Write updated ODR and BW (data[0]) to GYRO_CONF_ADDR
        self.write_u8(defs::BMI160_GYRO_CONF_ADDR, data[0])
            .map_err(Error::I2c)?;

        // --- Set Gyro Range ---
        data[1] =
            (data[1] & !defs::BMI160_GYRO_RANGE_MASK) | (range_val & defs::BMI160_GYRO_RANGE_MASK);

        // Write updated range (data[1]) to GYRO_RANGE_ADDR
        self.write_u8(defs::BMI160_GYRO_RANGE_ADDR, data[1])
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Read raw gyro data
    pub fn get_gyro_data(&mut self) -> Result<Vector3d<i16>, Error<E>> {
        let mut data_array: [u8; 6] = [0; 6];

        // Read gyro data (6 bytes) and optionally time (9 bytes)
        self.read_bytes(defs::BMI160_GYRO_DATA_ADDR, &mut data_array)
            .map_err(Error::I2c)?;

        // Parse gyroscope X, Y, Z data
        let x = i16::from_le_bytes([data_array[0], data_array[1]]);
        let y = i16::from_le_bytes([data_array[2], data_array[3]]);
        let z = i16::from_le_bytes([data_array[4], data_array[5]]);

        Ok(Vector3d { x, y, z })
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroConfig {
    pub odr: GyroODR,
    pub range: GyroRange,
    pub bw: GyroBandwidth,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroODR {
    Hz25 = 0x06,
    Hz50 = 0x07,
    Hz100 = 0x08,
    Hz200 = 0x09,
    Hz400 = 0x0A,
    Hz800 = 0x0B,
    Hz1600 = 0x0C,
    Hz3200 = 0x0D,
}

impl TryFrom<u8> for GyroODR {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x06 => Ok(GyroODR::Hz25),
            0x07 => Ok(GyroODR::Hz50),
            0x08 => Ok(GyroODR::Hz100),
            0x09 => Ok(GyroODR::Hz200),
            0x0A => Ok(GyroODR::Hz400),
            0x0B => Ok(GyroODR::Hz800),
            0x0C => Ok(GyroODR::Hz1600),
            0x0D => Ok(GyroODR::Hz3200),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroRange {
    Dps2000 = 0x00,
    Dps1000 = 0x01,
    Dps500 = 0x02,
    Dps250 = 0x03,
    Dps125 = 0x04,
}

impl TryFrom<u8> for GyroRange {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(GyroRange::Dps2000),
            0x01 => Ok(GyroRange::Dps1000),
            0x02 => Ok(GyroRange::Dps500),
            0x03 => Ok(GyroRange::Dps250),
            0x04 => Ok(GyroRange::Dps125),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroBandwidth {
    OSR4 = 0x00,
    OSR2 = 0x01,
    Normal = 0x02,
}

impl TryFrom<u8> for GyroBandwidth {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(GyroBandwidth::OSR4),
            0x01 => Ok(GyroBandwidth::OSR2),
            0x02 => Ok(GyroBandwidth::Normal),
            _ => Err(()),
        }
    }
}
