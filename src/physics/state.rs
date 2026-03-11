use bevy::prelude::*;

#[derive(Resource)]
pub struct VehicleState {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
    pub ax: f32,
    pub ay: f32,
    pub delta_fl: f32,
    pub delta_fr: f32,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            x: 10.0,
            y: 10.0,
            yaw: 0.0,
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
            ax: 0.0,
            ay: 0.0,
            delta_fl: 0.0,
            delta_fr: 0.0,
        }
    }
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
    pub h_cg: f32,
    pub c_rolling: f32,
    pub cd_a: f32,
    pub c_drivetrain: f32,
    pub c_omega_damp: f32,
    pub c_vy_damp: f32,
    pub rho_air: f32,
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
            h_cg: 0.04,
            c_rolling: 0.03,
            cd_a: 0.02,
            c_drivetrain: 0.2,
            c_omega_damp: 0.04,
            c_vy_damp: 1.0,
            rho_air: 1.225,
        }
    }
}

#[derive(Component)]
pub struct Robot;
