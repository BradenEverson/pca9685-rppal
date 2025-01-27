# PCA9685 RPPal

A [rppal](https://github.com/golemparts/rppal)-based Rust library for interacting with the [Adafruit PCA9685 16-channel PWM/Servo Driver](https://www.adafruit.com/product/815) on Raspberry Pi. This library wraps the lower-level I²C operations to simplify setting up the PCA9685 and controlling PWM channels.

## Features

- **Simple Initialization**: Easily create and configure the PCA9685 device at its default I²C address (`0x40`).
- **Frequency Control**: Set the PWM frequency (in Hz) on all 16 channels simultaneously.
- **Channel Control**: Independently control each channel by specifying ON/OFF counts (0–4095).
- **Bulk Control**: Set PWM for all channels at once.

## Getting Started

### Prerequisites

- A Raspberry Pi with I²C enabled (see [Raspberry Pi documentation](https://www.raspberrypi.org/documentation/configuration/i2c.md) for details).
- Rust (stable toolchain or later).
- [`rppal`](https://docs.rs/rppal) crate (installed automatically as a dependency).

### Installation

Add the following to your project's `Cargo.toml`:

```toml
[dependencies]
pca9685-rppal = "0.1.0"
```

## Usage

You can initialize a device, set the PWM frequency and control a single or all channels at once!

```rust

let mut pca = Pca9685::new()?;
pca.init()?;
pca.set_pwm_freq(50.0)?;

pca.set_pwm(0, 0, 1500)?;
```

```rust
pca9685.set_all_pwm(0, 0)?;
```
