use bevy::prelude::*;

#[derive(Resource)]
pub struct VehicleInput {
    pub steering: f32,
    pub throttle: f32,
}

impl Default for VehicleInput {
    fn default() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
        }
    }
}

pub fn read_input(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut input: ResMut<VehicleInput>,
) {
    if keys.pressed(KeyCode::ShiftLeft) {
        return;
    }

    let dt = time.delta_secs();
    let throttle_rate = 3.0;
    let steering_rate = 2.0;

    let target_throttle = if keys.pressed(KeyCode::KeyW) {
        8.0
    } else if keys.pressed(KeyCode::KeyS) {
        -8.0
    } else {
        0.0
    };

    let target_steering = if keys.pressed(KeyCode::KeyA) {
        -0.2
    } else if keys.pressed(KeyCode::KeyD) {
        0.2
    } else {
        0.0
    };

    let diff_t = target_throttle - input.throttle;
    input.throttle += diff_t.clamp(-throttle_rate * dt, throttle_rate * dt);

    let diff_s = target_steering - input.steering;
    input.steering += diff_s.clamp(-steering_rate * dt, steering_rate * dt);
}
