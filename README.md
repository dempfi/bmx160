# Rust Bosch BMX 160 crate

This crate is `no_std` and doesn't rely on any external crates, however for vector operations, `micromath` is available as a feature.

## Usage example

```rs
use bmx160::{
  BMX160, power::PowerMode,
  accel::AccelConfig,
  interrupt::{IntChannel, IntPinConfig, IntType},
  motion::{AnyMotionDuration, AnyMotionIntConf},
  tap::TapIntConf,
};
use defmt::info;
use embassy_nrf::{peripherals, twim::Twim};
use embassy_time::Timer;

pub type IMU = BMX160<Twim<'static, peripherals::SERIAL0>>;

pub async fn init_imu(mut i2c: Twim<'static, peripherals::SERIAL0>) -> IMU {
  Timer::after_millis(1000).await;

  i2c_scan(&mut i2c).await;

  let mut imu = BMX160::new(i2c);
  imu.init(&mut embassy_time::Delay).unwrap();

  // Set power mode
  let power_mode = PowerMode {
    accel: bmx160::power::AccelPowerMode::LowPower,
    gyro: bmx160::power::GyroPowerMode::Suspend,
    mag: bmx160::power::MagPowerMode::Suspend,
  };
  imu
    .set_power_mode(power_mode, &mut embassy_time::Delay)
    .unwrap();

  let acc_conf = AccelConfig {
    odr: bmx160::accel::AccelODR::Hz50,
    range: bmx160::accel::AccelRange::G2,
    bandwidth: bmx160::accel::AccelBandwidth::Normal,
    undersample: true,
  };

  imu.set_accel_conf(acc_conf).unwrap();

  // Create the tap interrupt configuration for light taps
  let tap_config = TapIntConf {
    quiet: bmx160::tap::TapQuiet::Ms20, // Faster response (20ms)
    shock: bmx160::tap::TapShock::Ms50, // Short shock time (50ms)
    duration: bmx160::tap::TapDuration::Ms100, // Detection window for double tap (100ms)
    threshold: bmx160::tap::TapThreshold::Lvl13, // Low threshold for light taps
  };

  imu.set_tap_interrupt_conf(tap_config).unwrap();

  // Configure interrupt pin behavior (INT1)
  let pin_config = IntPinConfig {
    output_enable: true,
    mode: bmx160::interrupt::PinMode::PushPull,
    trigger_level: bmx160::interrupt::TriggerLevel::ActiveHigh,
    trigger_type: bmx160::interrupt::TriggerType::Edge,
    latch_mode: bmx160::interrupt::LatchMode::Ms10,
    input_enable: false,
  };

  // Apply pin configuration for INT1
  imu.set_int_pin_conf(IntChannel::Int1, pin_config).unwrap();

  let any_motion_config = AnyMotionIntConf::new(AnyMotionDuration::One, 10);
  imu
    .set_any_motion_interrupt_conf(any_motion_config)
    .unwrap();

  // Enable and map the single tap interrupt to INT1
  imu
    .enable_int(IntChannel::Int1, IntType::SingleTap)
    .unwrap();

  // Enable and map the single tap interrupt to INT1
  imu
    .enable_int(IntChannel::Int1, IntType::AnyMotion)
    .unwrap();

  info!("{:?}", imu.get_power_mode().unwrap());

  imu
}
```
