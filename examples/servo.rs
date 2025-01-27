//! Example that runs a bunch of servos

use pca9685_rppal::Pca9685;

/// Minimum pulse length
const SERVO_MIN: u16 = 150;
/// Maximum pulse length
const SERVO_MAX: u16 = 600;

fn map_angle_to_pulse(angle: u16, servomin: u16, servomax: u16) -> u16 {
    let input_min = 0;
    let input_max = 180;
    servomin + (angle - input_min) * (servomax - servomin) / (input_max - input_min)
}

fn move_servo(pca: &mut Pca9685, idx: u8, angle: u16) -> rppal::i2c::Result<()> {
    let len = map_angle_to_pulse(angle, SERVO_MIN, SERVO_MAX);
    pca.set_pwm(idx, 0, len)?;

    Ok(())
}

fn main() {
    let mut pca = Pca9685::new().expect("Create Pca9685");
    pca.init().expect("Initialize PCA9685");
    pca.set_pwm_freq(50.0)
        .expect("Set frequency to 50hz for servos");

    let standing_angles = [87, 80, 86, 95, 90, 66, 95, 80, 76, 85, 80, 72];

    for (idx, angle) in standing_angles.iter().enumerate() {
        println!("Moving servo {idx} to {angle}");
        move_servo(&mut pca, idx as u8, *angle).expect("Move dog");
    }
}
