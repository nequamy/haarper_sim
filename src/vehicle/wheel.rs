use crate::vehicle::input::VehicleInput;
use bevy::prelude::*;

#[derive(Component)]
pub struct Wheel;

#[derive(Component)]
pub struct WheelF;

#[derive(Resource)]
pub struct WheelAngleSpeed {
    pub angle: f32,
}

impl Default for WheelAngleSpeed {
    fn default() -> Self {
        Self { angle: 0.0 }
    }
}

pub fn update_forward_wheel_angle(
    input: Res<VehicleInput>,
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<WheelF>>,
) {
    let _ = query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(-input.steering + std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}

pub fn update_wheel_angle(
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<Wheel>>,
) {
    let _ = query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}
