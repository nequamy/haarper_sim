use bevy::prelude::*;

#[derive(Resource)]
pub struct MotorParams {
    pub ke: f32,
    pub kt: f32,
    pub r: f32,
    pub i_max: f32,
    pub gear_ratio: f32,
    pub eta: f32,
}

impl Default for MotorParams {
    fn default() -> Self {
        Self {
            ke: 0.001552,
            kt: 0.00233,
            r: 0.0059,
            i_max: 30.0,
            gear_ratio: 10.64,
            eta: 0.85,
        }
    }
}

#[derive(Resource, Default)]
pub struct MotorState {
    pub current: f32,
    pub rpm: f32,
    pub voltage: f32,
    pub duty: f32,
}

impl MotorState {
    pub fn compute(
        &mut self,
        duty: f32,
        velocity: f32,
        radius: f32,
        voltage: f32,
        params: &MotorParams,
    ) -> f32 {
        self.duty = duty.clamp(-1.0, 1.0);

        self.voltage = self.duty * voltage;
        let v_bemf = params.ke * (velocity / radius) * params.gear_ratio;

        if duty != 0.0 {
            self.current = ((self.voltage - v_bemf) / params.r).clamp(-params.i_max, params.i_max);
        } else {
            self.current = 0.0;
        }

        if duty.signum() != self.current.signum() {
            self.current = 0.0;
        }

        self.rpm = (velocity / radius) * params.gear_ratio * 60.0 / (2.0 * std::f32::consts::PI);

        params.kt * self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_compute_on_zero_duty() {
        let mut motor = MotorState::default();

        let t = motor.compute(0.0, 0.0, 0.055, 12.6, &MotorParams::default());

        assert_eq!(t, 0.0);
        assert_eq!(motor.voltage, 0.0);
        assert_eq!(motor.current, 0.0);
        assert_eq!(motor.duty, 0.0);
    }

    #[test]
    fn test_motor_compute_on_max_duty() {
        let mut motor = MotorState::default();
        let params = MotorParams::default();

        let t = motor.compute(1.0, 0.0, 0.055, 12.6, &params);

        assert!(t <= 11.5);
        assert!(motor.voltage == 12.6);
        assert!(motor.current <= params.i_max);
        assert!(motor.duty == 1.0);
    }

    #[test]
    fn test_motor_compute_on_min_duty() {
        let mut motor = MotorState::default();
        let params = MotorParams::default();

        let t = motor.compute(-1.0, 0.0, 0.055, 12.6, &params);

        assert!(t >= -11.5);
        assert!(motor.voltage == -12.6);
        assert!(motor.current <= -params.i_max);
        assert!(motor.duty == -1.0);
    }

    #[test]
    fn test_motor_compute_on_velocity() {
        let mut motor = MotorState::default();
        let params = MotorParams::default();

        let t = motor.compute(0.02, 40.0, 0.055, 12.6, &params);

        assert!(t <= 11.5);
        assert!(motor.voltage <= 2.0);
        assert!(motor.current >= -params.i_max);
        assert!(motor.duty == 0.02);
    }
}
