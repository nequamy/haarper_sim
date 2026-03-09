use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::physics::state::{Robot, VehicleParams, VehicleState};
use crate::vehicle::input::VehicleInput;
use crate::vehicle::wheel::WheelAngleSpeed;

pub fn update_physics(
    time: Res<Time>,
    input: Res<VehicleInput>,
    params: Res<VehicleParams>,
    mut wheel_spd: ResMut<WheelAngleSpeed>,
    mut state: ResMut<VehicleState>,
) {
    let dt = time.delta_secs();

    let fdr: f32 = input.throttle * params.f_max;

    wheel_spd.angle += (state.vx / params.r_wheel) * dt;

    // Скорости
    let vx_fl = state.vx - (params.w / 2.0) * state.omega;
    let vy_fl = state.vy + params.lf * state.omega;
    let vx_fr = state.vx + (params.w / 2.0) * state.omega;
    let vy_fr = state.vy + params.lf * state.omega;
    let vx_rl = state.vx - (params.w / 2.0) * state.omega;
    let vy_rl = state.vy - params.lr * state.omega;
    let vx_rr = state.vx + (params.w / 2.0) * state.omega;
    let vy_rr = state.vy - params.lr * state.omega;

    // Slip angle
    let vx_fl_w = vx_fl * input.steering.cos() + vy_fl * input.steering.sin();
    let vy_fl_w = -vx_fl * input.steering.sin() + vy_fl * input.steering.cos();
    let vx_fr_w = vx_fr * input.steering.cos() + vy_fr * input.steering.sin();
    let vy_fr_w = -vx_fr * input.steering.sin() + vy_fr * input.steering.cos();

    let alpha_fl = safe_slip_angle(vy_fl_w, vx_fl_w);
    let alpha_fr = safe_slip_angle(vy_fr_w, vx_fr_w);
    let alpha_rl = safe_slip_angle(vy_rl, vx_rl);
    let alpha_rr = safe_slip_angle(vy_rr, vx_rr);

    // Силы
    let ff_ly: f32 = (params.tire_mu * (params.m * 9.81 * params.lr / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_fl).atan()).sin();
    let ff_ry: f32 = (params.tire_mu * (params.m * 9.81 * params.lr / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_fr).atan()).sin();
    let fr_ly: f32 = (params.tire_mu * (params.m * 9.81 * params.lf / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_rl).atan()).sin();
    let fr_ry: f32 = (params.tire_mu * (params.m * 9.81 * params.lf / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_rr).atan()).sin();

    let fx_fl_body = (fdr / 4.0) * input.steering.cos() - ff_ly * input.steering.sin();
    let fy_fl_body = (fdr / 4.0) * input.steering.sin() + ff_ly * input.steering.cos();
    let fx_fr_body = (fdr / 4.0) * input.steering.cos() - ff_ry * input.steering.sin();
    let fy_fr_body = (fdr / 4.0) * input.steering.sin() + ff_ry * input.steering.cos();

    let fx_rl_body = fdr / 4.0;
    let fy_rl_body = fr_ly;
    let fx_rr_body = fdr / 4.0;
    let fy_rr_body = fr_ry;

    // Сопротивления
    let f_rolling = (0.015 * params.m * 9.81) * state.vx.signum();
    let f_aero = 0.5 * 1.225 * 0.4 * 0.02 * state.vx * state.vx.abs();
    let f_drivetrain = 1.0 * state.vx;
    let omega_damp = 0.04 * state.omega;
    let vy_damp = 1.0 * state.vy;
    let total_resist = f_rolling + f_aero + f_drivetrain;

    let sum_fx = fx_fl_body + fx_fr_body + fx_rl_body + fx_rr_body - total_resist;
    let sum_fy = fy_fl_body + fy_fr_body + fy_rl_body + fy_rr_body - vy_damp;
    let sum_m = params.lf * (fy_fl_body + fy_fr_body) - params.lr * (fy_rl_body + fy_rr_body)
        + (params.w / 2.0) * (-fx_fl_body + fx_fr_body - fx_rl_body + fx_rr_body)
        - omega_damp;

    let vx_old = state.vx;
    let vy_old = state.vy;

    state.vx += ((1.0 / params.m) * (sum_fx + params.m * vy_old * state.omega)) * dt;
    state.vy += ((1.0 / params.m) * (sum_fy - params.m * vx_old * state.omega)) * dt;
    state.omega += (1.0 / params.iz) * sum_m * dt;

    state.x += (state.vx * state.yaw.cos() - state.vy * state.yaw.sin()) * dt;
    state.y += (state.vx * state.yaw.sin() + state.vy * state.yaw.cos()) * dt;
    state.yaw += state.omega * dt;
}

pub fn sync_vehicle_transform(
    state: Res<VehicleState>,
    mut query: Query<(&mut Transform, &mut KinematicCharacterController), With<Robot>>,
) {
    let Ok((mut tf, mut controller)) = query.single_mut() else {
        return;
    };

    controller.translation = Some(Vec3::new(
        state.x - tf.translation.x,
        0.0,
        state.y - tf.translation.z,
    ));

    tf.rotation = Quat::from_rotation_y(-state.yaw);
}

fn safe_slip_angle(vy: f32, vx: f32) -> f32 {
    if vx.abs() < 0.5 {
        -(vy * 2.0).clamp(-1.0, 1.0)
    } else {
        -(vy / vx).atan()
    }
}
