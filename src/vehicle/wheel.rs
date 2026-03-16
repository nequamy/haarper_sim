use crate::physics::state::VehicleState;
use bevy::prelude::*;

#[derive(Component)]
pub struct Wheel;

#[derive(Component)]
pub struct WheelFR;

#[derive(Component)]
pub struct WheelFL;

#[derive(Resource)]
pub struct WheelAngleSpeed {
    pub angle: f32,
}

#[derive(Resource)]
pub struct WheelDynamics {
    pub omega_fl: f32,
    pub omega_fr: f32,
    pub omega_rl: f32,
    pub omega_rr: f32,
    pub j_eff: f32,
    pub kappa: [f32; 4],
    pub alpha: [f32; 4],
}

impl WheelDynamics {
    pub fn new() -> Self {
        Self {
            omega_fl: 0.0,
            omega_fr: 0.0,
            omega_rl: 0.0,
            omega_rr: 0.0,
            j_eff: 0.0012,
            kappa: [0.0, 0.0, 0.0, 0.0],
            alpha: [0.0, 0.0, 0.0, 0.0],
        }
    }
}

impl Default for WheelAngleSpeed {
    fn default() -> Self {
        Self { angle: 0.0 }
    }
}

pub fn update_forward_left_wheel_angle(
    state: Res<VehicleState>,
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<WheelFL>>,
) {
    query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(-state.delta_fl + std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}

pub fn update_forward_right_wheel_angle(
    state: Res<VehicleState>,
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<WheelFR>>,
) {
    query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(-state.delta_fr + std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}
pub fn update_wheel_angle(
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<Wheel>>,
) {
    query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}
