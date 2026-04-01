use bevy::prelude::*;

use crate::{
    physics::{
        battery::BatteryState,
        motor::MotorState,
        state::{Robot, VehicleState},
    },
    track::TrackData,
    ui::resources::TrailHistory,
    vehicle::{
        input::VehicleInput,
        spawn::{get_start_heading, get_start_position},
        wheel::WheelDynamics,
    },
};

pub fn reset_world(
    key: Res<ButtonInput<KeyCode>>,
    track: Res<TrackData>,
    mut vehicle: ResMut<VehicleState>,
    mut wheels: ResMut<WheelDynamics>,
    mut battery: ResMut<BatteryState>,
    mut motor: ResMut<MotorState>,
    mut input: ResMut<VehicleInput>,
    mut query: Query<&mut Transform, With<Robot>>,
    mut track_history: ResMut<TrailHistory>,
) {
    if !key.just_pressed(KeyCode::KeyR) {
        return;
    }

    let Ok(mut tf) = query.single_mut() else {
        return;
    };

    let (x, y, z) = get_start_position(&track);
    let yaw = get_start_heading(&track);

    *vehicle = VehicleState::default();
    vehicle.x = x;
    vehicle.y = z;
    vehicle.yaw = yaw;

    tf.translation = Vec3::new(x, y, z);
    tf.rotation = Quat::from_rotation_y(-yaw);

    *input = VehicleInput::default();

    wheels.omega_fl = 0.0;
    wheels.omega_fr = 0.0;
    wheels.omega_rl = 0.0;
    wheels.omega_rr = 0.0;

    *battery = BatteryState::new();
    *motor = MotorState::default();

    track_history.trail.clear();
}
