//! Rppal Wrapper for the Adafruit PCA9685 Servo/PWM Controller

use rppal::i2c::I2c;

// Register definitions.

/// The default I²C address of the PCA9685 device.
const PCA9685_ADDRESS: u16 = 0x40;
/// Mode 1 register address, used for configuring basic operation modes of the PCA9685.
const MODE1: u8 = 0x00;
/// Mode 2 register address, used for configuring output behavior of the PCA9685.
const MODE2: u8 = 0x01;
/// Subaddress 1 register, used to set the first alternative I²C subaddress.
const SUBADR1: u8 = 0x02;
/// Subaddress 2 register, used to set the second alternative I²C subaddress.
const SUBADR2: u8 = 0x03;
/// Subaddress 3 register, used to set the third alternative I²C subaddress.
const SUBADR3: u8 = 0x04;
/// Prescale register, used to configure the PWM frequency of the PCA9685.
const PRESCALE: u8 = 0xFE;
/// LED0_ON_L register address, low byte of the LED0 ON time.
const LED0_ON_L: u8 = 0x06;
/// LED0_ON_H register address, high byte of the LED0 ON time.
const LED0_ON_H: u8 = 0x07;
/// LED0_OFF_L register address, low byte of the LED0 OFF time.
const LED0_OFF_L: u8 = 0x08;
/// LED0_OFF_H register address, high byte of the LED0 OFF time.
const LED0_OFF_H: u8 = 0x09;
/// ALL_LED_ON_L register address, low byte for turning all LEDs ON.
const ALL_LED_ON_L: u8 = 0xFA;
/// ALL_LED_ON_H register address, high byte for turning all LEDs ON.
const ALL_LED_ON_H: u8 = 0xFB;
/// ALL_LED_OFF_L register address, low byte for turning all LEDs OFF.
const ALL_LED_OFF_L: u8 = 0xFC;
/// ALL_LED_OFF_H register address, high byte for turning all LEDs OFF.
const ALL_LED_OFF_H: u8 = 0xFD;

// Bit definitions.

/// Bit mask for restarting the PCA9685 oscillator and resetting its state.
const RESTART: u8 = 0x80;
/// Bit mask for enabling low-power sleep mode.
const SLEEP: u8 = 0x10;
/// Bit mask for enabling the ALLCALL address, allowing all devices to respond to a general call.
const ALLCALL: u8 = 0x01;
/// Bit mask for inverting the output logic of the PCA9685.
const INVRT: u8 = 0x10;
/// Bit mask for setting the output driver mode to totem-pole (instead of open-drain).
const OUTDRV: u8 = 0x04;

// Other constants.

/// Software reset command for the PCA9685, used to reset all devices on the I²C bus.
const SWRST: u8 = 0x06;

/// A PCA9685 device
pub struct Pca9685 {
    /// Underlying I2c Device
    i2c: I2c,
}
