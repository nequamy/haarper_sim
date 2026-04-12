use bevy::{math::FloatPow, prelude::*};

#[derive(Resource)]
pub struct ServoParams {
    pub omega_n: f32,
    pub zeta: f32,
    pub omega_max: f32,
    pub delta_max: f32,
}

impl Default for ServoParams {
    fn default() -> Self {
        Self {
            omega_n: 17.0,
            zeta: 1.2,
            omega_max: 10.5,
            delta_max: 0.436,
        }
    }
}

#[derive(Resource, Default)]
pub struct ServoState {
    /// Текущий угол
    pub delta: f32,
    /// Текущая скорость
    pub delta_dot: f32,
}

impl ServoState {
    pub fn update(&mut self, delta_cmd: f32, dt: f32, params: &ServoParams) {
        let accel = params.omega_n.squared() * (delta_cmd - self.delta)
            - 2.0 * params.zeta * params.omega_n * self.delta_dot;

        self.delta_dot += accel * dt;
        self.delta_dot = self.delta_dot.clamp(-params.omega_max, params.omega_max);

        self.delta += self.delta_dot * dt;
        self.delta = self.delta.clamp(-params.delta_max, params.delta_max);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_servo_steady_state() {
        let params = ServoParams::default();
        let mut servo = ServoState::default();

        for _ in 0..=5000 {
            servo.update(0.2, 0.002, &params);
        }

        assert!((servo.delta - 0.2).abs() <= 0.001);
    }

    #[test]
    fn test_servo_rate_limit() {
        let params = ServoParams::default();
        let mut servo = ServoState::default();

        for _ in 0..=1000 {
            servo.update(0.2, 0.002, &params);
        }

        assert!(servo.delta_dot.abs() > 0.0);
        assert!(servo.delta_dot.abs() <= params.omega_max);
    }

    #[test]
    fn test_servo_position_limit() {
        let params = ServoParams::default();
        let mut servo = ServoState::default();

        for _ in 0..=1000 {
            servo.update(0.2, 0.002, &params);
        }

        assert!(servo.delta.abs() > 0.0);
        assert!(servo.delta.abs() <= params.delta_max);
    }
}
