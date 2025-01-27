//! A `rppal`-based wrapper for the Adafruit PCA9685 Servo/PWM Controller.
//!
//! This crate provides a convenient interface to the PCA9685 PWM driver via I²C on the Raspberry Pi,
//! using the [`rppal`](https://docs.rs/rppal) library.
//!
//! The PCA9685 is often used to control servos or LEDs, offering up to 16 independent PWM channels.
//!
//! # Example
//!
//! ```ignore
//! use pca9685_rppal::*;
//!
//! let mut pwm = Pca9685::new().expect("Create Pca9685");
//! pwm.init().expect("Initialize Pca9685");
//! pwm.set_pwm_freq(50.0).expect("Set Frequency to 50hz (common for servos)");
//! pwm.set_pwm(0, 0, 1500).expect("Set PWM on channel 0");
//!
//! ```

use std::time::Duration;

use rppal::i2c::I2c;

// Register definitions.

/// The default I²C address of the PCA9685 device.
pub const PCA9685_ADDRESS: u16 = 0x40;
/// Mode 1 register address, used for configuring basic operation modes of the PCA9685.
pub const MODE1: u8 = 0x00;
/// Mode 2 register address, used for configuring output behavior of the PCA9685.
pub const MODE2: u8 = 0x01;
/// Subaddress 1 register, used to set the first alternative I²C subaddress.
pub const SUBADR1: u8 = 0x02;
/// Subaddress 2 register, used to set the second alternative I²C subaddress.
pub const SUBADR2: u8 = 0x03;
/// Subaddress 3 register, used to set the third alternative I²C subaddress.
pub const SUBADR3: u8 = 0x04;
/// Prescale register, used to configure the PWM frequency of the PCA9685.
pub const PRESCALE: u8 = 0xFE;
/// LED0_ON_L register address, low byte of the LED0 ON time.
pub const LED0_ON_L: u8 = 0x06;
/// LED0_ON_H register address, high byte of the LED0 ON time.
pub const LED0_ON_H: u8 = 0x07;
/// LED0_OFF_L register address, low byte of the LED0 OFF time.
pub const LED0_OFF_L: u8 = 0x08;
/// LED0_OFF_H register address, high byte of the LED0 OFF time.
pub const LED0_OFF_H: u8 = 0x09;
/// ALL_LED_ON_L register address, low byte for turning all LEDs ON.
pub const ALL_LED_ON_L: u8 = 0xFA;
/// ALL_LED_ON_H register address, high byte for turning all LEDs ON.
pub const ALL_LED_ON_H: u8 = 0xFB;
/// ALL_LED_OFF_L register address, low byte for turning all LEDs OFF.
pub const ALL_LED_OFF_L: u8 = 0xFC;
/// ALL_LED_OFF_H register address, high byte for turning all LEDs OFF.
pub const ALL_LED_OFF_H: u8 = 0xFD;

// Bit definitions.

/// Bit mask for restarting the PCA9685 oscillator and resetting its state.
pub const RESTART: u8 = 0x80;
/// Bit mask for enabling low-power sleep mode.
pub const SLEEP: u8 = 0x10;
/// Bit mask for enabling the ALLCALL address, allowing all devices to respond to a general call.
pub const ALLCALL: u8 = 0x01;
/// Bit mask for inverting the output logic of the PCA9685.
pub const INVRT: u8 = 0x10;
/// Bit mask for setting the output driver mode to totem-pole (instead of open-drain).
pub const OUTDRV: u8 = 0x04;

// Other constants.

/// Software reset command for the PCA9685, used to reset all devices on the I²C bus.
pub const SWRST: u8 = 0x06;

/// Represents a PCA9685 device connected via I²C.
///
/// This struct wraps an `rppal::i2c::I2c` instance pointed at the
/// appropriate slave address, providing helper methods to easily
/// configure the PCA9685 and set PWM values on its channels.
pub struct Pca9685 {
    /// Underlying I²c Device
    i2c: I2c,
}

impl Pca9685 {
    /// Constructs a new `Pca9685` device at the default address (0x40) on the default I²C bus.
    ///
    /// # Errors
    ///
    /// Returns an error of type [`rppal::i2c::Error`] if an I²C device
    /// cannot be created or the default bus cannot be accessed.
    pub fn new() -> rppal::i2c::Result<Self> {
        let mut i2c = I2c::new()?;
        i2c.set_slave_address(PCA9685_ADDRESS)?;

        Ok(Self { i2c })
    }

    /// Initializes the PCA9685 for standard operation.
    ///
    /// 1. Sets all PWM outputs to "off".
    /// 2. Configures the device to use the totem-pole driver and enable the ALLCALL address.
    /// 3. Takes the PCA9685 out of sleep mode.
    ///
    /// # Errors
    ///
    /// Returns an error if any I²C write or read operations fail.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut pca = Pca9685::new()?;
    /// pca.init()?;
    /// ```
    pub fn init(&mut self) -> rppal::i2c::Result<()> {
        self.set_all_pwm(0, 0)?;

        self.i2c.smbus_write_byte(MODE2, OUTDRV)?;
        self.i2c.smbus_write_byte(MODE1, ALLCALL)?;
        std::thread::sleep(Duration::from_millis(5));

        let mode1 = self.i2c.smbus_read_byte(MODE1)?;
        let mode1 = mode1 & !SLEEP;

        self.i2c.smbus_write_byte(MODE1, mode1)?;
        std::thread::sleep(Duration::from_millis(5));

        Ok(())
    }

    /// Sets the PWM frequency (in Hertz).
    ///
    /// The PCA9685's internal clock is assumed to run at 25 MHz. The prescaler is computed to
    /// achieve the target frequency, then the device is put to sleep momentarily while the new
    /// prescaler is written.
    ///
    /// # Arguments
    ///
    /// * `freq_hz` - Desired frequency in Hertz (e.g., `50.0` for servos).
    ///
    /// # Errors
    ///
    /// Returns an error if any I²C write or read operations fail.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut pca = Pca9685::new()?;
    /// pca.init()?;
    /// pca.set_pwm_freq(60.0)?; // typical for some servos
    /// ```
    pub fn set_pwm_freq(&mut self, freq_hz: f32) -> rppal::i2c::Result<()> {
        let prescaleval = 25e6 / 4096.0 / freq_hz - 1.0;
        let prescale = (prescaleval + 0.5).floor() as u8;

        let old_mode = self.i2c.smbus_read_byte(MODE1)?;
        let new_mode = old_mode & 0x7F | SLEEP;

        self.i2c.smbus_write_byte(MODE1, new_mode)?;
        self.i2c.smbus_write_byte(PRESCALE, prescale)?;
        self.i2c.smbus_write_byte(MODE1, old_mode)?;
        std::thread::sleep(Duration::from_millis(5));

        self.i2c.smbus_write_byte(MODE1, old_mode | 0xA1)
    }

    /// Sets the PWM ON and OFF counts for a single channel.
    ///
    /// # Arguments
    ///
    /// * `channel` - The channel index (0–15) on the PCA9685.
    /// * `on` - The timer tick at which the output is switched ON.
    /// * `off` - The timer tick at which the output is switched OFF.
    ///
    /// Each channel output is controlled by two 12-bit registers: ON and OFF. The timer counts from 0 to 4095.
    ///
    /// # Errors
    ///
    /// Returns an error if any I²C write operation fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut pca = Pca9685::new()?;
    /// pca.init()?;
    /// // Turn channel 0 on at 0, off at 1500
    /// pca.set_pwm(0, 0, 1500)?;
    /// ```
    pub fn set_pwm(&mut self, channel: u8, on: u16, off: u16) -> rppal::i2c::Result<()> {
        self.i2c
            .smbus_write_byte(LED0_ON_L + 4 * channel, (on & 0xFF) as u8)?;

        self.i2c
            .smbus_write_byte(LED0_ON_H + 4 * channel, (on >> 8) as u8)?;

        self.i2c
            .smbus_write_byte(LED0_OFF_L + 4 * channel, (off & 0xFF) as u8)?;

        self.i2c
            .smbus_write_byte(LED0_OFF_H + 4 * channel, (off >> 8) as u8)
    }

    /// Sets the PWM ON and OFF counts for *all* channels simultaneously.
    ///
    /// # Arguments
    ///
    /// * `on` - The timer tick at which all outputs are switched ON.
    /// * `off` - The timer tick at which all outputs are switched OFF.
    ///
    /// # Errors
    ///
    /// Returns an error if any I²C write operation fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut pca = Pca9685::new()?;
    /// pca.init()?;
    /// // Turn all channels fully OFF
    /// pca.set_all_pwm(0, 0)?;
    /// ```
    pub fn set_all_pwm(&mut self, on: u16, off: u16) -> rppal::i2c::Result<()> {
        self.i2c.smbus_write_byte(ALL_LED_ON_L, (on & 0xFF) as u8)?;

        self.i2c.smbus_write_byte(ALL_LED_ON_H, (on >> 8) as u8)?;

        self.i2c
            .smbus_write_byte(ALL_LED_OFF_L, (off & 0xFF) as u8)?;

        self.i2c.smbus_write_byte(ALL_LED_OFF_H, (off >> 8) as u8)
    }
}
