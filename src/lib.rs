//! Rppal Wrapper for the Adafruit PCA9685 Servo/PWM Controller

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

/// A PCA9685 device
pub struct Pca9685 {
    /// Underlying I2c Device
    i2c: I2c,
}

impl Pca9685 {
    /// Construct a new Pca9685 device
    pub fn new() -> rppal::i2c::Result<Self> {
        let mut i2c = I2c::new()?;
        i2c.set_slave_address(PCA9685_ADDRESS)?;

        Ok(Self { i2c })
    }

    /// Initialize device
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

    /// Set the PWM at a specific channel
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
    /// Set the PWM at all channels
    pub fn set_all_pwm(&mut self, on: u16, off: u16) -> rppal::i2c::Result<()> {
        self.i2c.smbus_write_byte(ALL_LED_ON_L, (on & 0xFF) as u8)?;

        self.i2c.smbus_write_byte(ALL_LED_ON_H, (on >> 8) as u8)?;

        self.i2c
            .smbus_write_byte(ALL_LED_OFF_L, (off & 0xFF) as u8)?;

        self.i2c.smbus_write_byte(ALL_LED_OFF_H, (off >> 8) as u8)
    }
}
