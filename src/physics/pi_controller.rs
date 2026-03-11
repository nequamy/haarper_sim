use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct PiController {
    pub kp: f32,
    pub ki: f32,
    pub integral: f32,
    pub saturation: f32,
    pub target_velocity: f32,
}

impl PiController {
    pub fn update(&mut self, current_velocity: f32, dt: f32) -> f32 {
        let error = self.target_velocity - current_velocity;

        self.integral += error * dt;
        self.integral = self.integral.clamp(
            -self.saturation / self.ki,
            self.saturation / self.ki.max(f32::EPSILON),
        );

        let mut output = self.kp * error + self.ki * self.integral;

        output = output.clamp(-self.saturation, self.saturation);
        // output = output.clamp(0.0, self.saturation);

        output
    }
}
