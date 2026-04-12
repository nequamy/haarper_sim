use bevy::prelude::*;
use rand::rng;
use rand_distr::{Distribution, Normal};

use crate::physics::state::VehicleState;

#[derive(Resource)]
pub struct ImuParams {
    pub accel_noise_std: f32,
    pub gyro_noise_std: f32,
    pub accel_bias_walk: f32,
    pub gyro_bias_walk: f32,
    pub timer: Timer,
}

impl Default for ImuParams {
    fn default() -> Self {
        Self {
            accel_noise_std: 0.015,
            gyro_noise_std: 0.0024,
            accel_bias_walk: 0.0001,
            gyro_bias_walk: 0.00002,
            timer: Timer::from_seconds(0.01, TimerMode::Repeating),
        }
    }
}

#[derive(Resource, Default)]
pub struct ImuData {
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    pub quaternion: [f32; 4],
    pub timestamp: f64,
}

#[derive(Resource, Default)]
pub struct ImuBias {
    accel_bias: [f32; 3],
    gyro_bias: [f32; 3],
}

pub fn simulate_imu(
    state: Res<VehicleState>,
    time: Res<Time>,
    mut bias: ResMut<ImuBias>,
    mut params: ResMut<ImuParams>,
    mut data: ResMut<ImuData>,
) {
    params.timer.tick(time.delta());
    if !params.timer.just_finished() {
        return;
    }

    let dt = params.timer.duration().as_secs_f32();
    let mut rng = rng();

    bias.accel_bias[0] +=
        Normal::new(0.0, 1.0).unwrap().sample(&mut rng) * params.accel_bias_walk * dt.sqrt();
    bias.accel_bias[1] +=
        Normal::new(0.0, 1.0).unwrap().sample(&mut rng) * params.accel_bias_walk * dt.sqrt();
    bias.accel_bias[2] +=
        Normal::new(0.0, 1.0).unwrap().sample(&mut rng) * params.accel_bias_walk * dt.sqrt();

    bias.gyro_bias[0] +=
        Normal::new(0.0, 1.0).unwrap().sample(&mut rng) * params.gyro_bias_walk * dt.sqrt();
    bias.gyro_bias[1] +=
        Normal::new(0.0, 1.0).unwrap().sample(&mut rng) * params.gyro_bias_walk * dt.sqrt();
    bias.gyro_bias[2] +=
        Normal::new(0.0, 1.0).unwrap().sample(&mut rng) * params.gyro_bias_walk * dt.sqrt();

    data.accel[0] = state.ax
        + bias.accel_bias[0]
        + Normal::new(0.0, params.accel_noise_std)
            .unwrap()
            .sample(&mut rng);
    data.accel[1] = state.ay
        + bias.accel_bias[1]
        + Normal::new(0.0, params.accel_noise_std)
            .unwrap()
            .sample(&mut rng);
    data.accel[2] = 9.81
        + bias.accel_bias[2]
        + Normal::new(0.0, params.accel_noise_std)
            .unwrap()
            .sample(&mut rng);

    data.gyro[0] = 0.0
        + bias.gyro_bias[0]
        + Normal::new(0.0, params.gyro_noise_std)
            .unwrap()
            .sample(&mut rng);
    data.gyro[1] = 0.0
        + bias.gyro_bias[1]
        + Normal::new(0.0, params.gyro_noise_std)
            .unwrap()
            .sample(&mut rng);
    data.gyro[2] = state.omega
        + bias.gyro_bias[2]
        + Normal::new(0.0, params.gyro_noise_std)
            .unwrap()
            .sample(&mut rng);

    let quat = Quat::from_rotation_y(-state.yaw);
    data.quaternion = [quat.x, quat.y, quat.z, quat.w];
    data.timestamp = time.elapsed_secs_f64();
}
