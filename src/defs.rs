#![allow(dead_code)]
/// BMI160 unique chip identifier
pub const BMX160_CHIP_ID: u8 = 0xD8;

/// Mask definitions
pub const BMI160_ACCEL_BW_MASK: u8 = 0x70;
pub const BMI160_ACCEL_US_MASK: u8 = 0b1000000;
pub const BMI160_ACCEL_ODR_MASK: u8 = 0x0F;
pub const BMI160_ACCEL_UNDERSAMPLING_MASK: u8 = 0x80;
pub const BMI160_ACCEL_RANGE_MASK: u8 = 0x0F;
pub const BMI160_GYRO_BW_MASK: u8 = 0x30;
pub const BMI160_GYRO_ODR_MASK: u8 = 0x0F;
pub const BMI160_GYRO_RANGE_MASK: u8 = 0x07;

pub const BMI160_ACCEL_BW_POS: u8 = 4;
pub const BMI160_GYRO_BW_POS: u8 = 4;

/// Registers map
pub const BMX160_I2C_ADDR: u8 = 0x68;
pub const BMX160_CHIP_ID_ADDR: u8 = 0x00;
pub const BMI160_PMU_STATUS_ADDR: u8 = 0x03;
pub const BMI160_ACCEL_DATA_ADDR: u8 = 0x12;
pub const BMI160_GYRO_DATA_ADDR: u8 = 0x0C;
pub const BMI160_ACCEL_CONF_ADDR: u8 = 0x40;
pub const BMI160_ACCEL_RANGE_ADDR: u8 = 0x41;
pub const BMI160_GYRO_CONF_ADDR: u8 = 0x42;
pub const BMI160_GYRO_RANGE_ADDR: u8 = 0x43;
pub const BMI160_INT_DATA_0_ADDR: u8 = 0x58;
pub const BMI160_COMMAND_REG_ADDR: u8 = 0x7E;
pub const BMI160_INT_EN_ADDR: u8 = 0x50;
pub const BMI160_INT_OUT_CTRL_ADDR: u8 = 0x53;
pub const BMI160_INT_MAP_ADDR: u8 = 0x55;
pub const BMI160_INT_TAP_ADDR: u8 = 0x63;
pub const BMI160_INT_STATUS_ADDR: u8 = 0x1C;
pub const BMI160_INT_MOTION_ADDR: u8 = 0x5F;

/// Soft reset command
pub const BMI160_SOFT_RESET_CMD: u8 = 0xb6;
pub const BMI160_SOFT_RESET_DELAY_MS: u8 = 1;

/// Power mode masks
pub const BMI160_ACCEL_POWER_MODE_MSK: u8 = 0b00110000; // Bits 4-5
pub const BMI160_ACCEL_POWER_MODE_POS: u8 = 4;

pub const BMI160_GYRO_POWER_MODE_MSK: u8 = 0b00001100; // Bits 2-3
pub const BMI160_GYRO_POWER_MODE_POS: u8 = 2;

pub const BMI160_MAG_POWER_MODE_MSK: u8 = 0b00000011; // Bits 0-1
pub const BMI160_MAG_POWER_MODE_POS: u8 = 0;

pub const BMI160_ACCEL_DELAY_MS: u32 = 5;
pub const BMI160_GYRO_DELAY_MS: u32 = 80;

/// Mask definitions for INT_OUT_CTRL register
pub const BMI160_INT1_EDGE_CTRL_MASK: u8 = 0x01;
pub const BMI160_INT1_OUTPUT_MODE_MASK: u8 = 0x04;
pub const BMI160_INT1_OUTPUT_TYPE_MASK: u8 = 0x02;
pub const BMI160_INT1_OUTPUT_EN_MASK: u8 = 0x08;
pub const BMI160_INT2_EDGE_CTRL_MASK: u8 = 0x10;
pub const BMI160_INT2_OUTPUT_MODE_MASK: u8 = 0x40;
pub const BMI160_INT2_OUTPUT_TYPE_MASK: u8 = 0x20;
pub const BMI160_INT2_OUTPUT_EN_MASK: u8 = 0x80;

/// Mask definitions for INT_LATCH register
pub const BMI160_INT1_INPUT_EN_MASK: u8 = 0x10;
pub const BMI160_INT2_INPUT_EN_MASK: u8 = 0x20;
pub const BMI160_INT_LATCH_MASK: u8 = 0x0F;
