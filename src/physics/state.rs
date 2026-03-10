use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct VehicleState {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
    pub ax: f32,
    pub ay: f32,
}

#[derive(Resource)]
pub struct VehicleParams {
    pub m: f32,
    pub iz: f32,
    pub lf: f32,
    pub lr: f32,
    pub wheelbase: f32,
    pub w: f32,
    pub r_wheel: f32,
    pub f_max: f32,
    pub h_cg: f32,
}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            m: 4.0,
            iz: 0.05,
            lf: 0.18,
            lr: 0.15,
            wheelbase: 0.33,
            w: 0.25,
            r_wheel: 0.055,
            f_max: 11.5,
            h_cg: 0.04,
        }
    }
}

#[derive(Component)]
pub struct Robot;
