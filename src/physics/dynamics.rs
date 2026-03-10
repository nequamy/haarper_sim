use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::physics::state::{Robot, VehicleParams, VehicleState};
use crate::physics::tire_model::{Pacejka, TireParams};
use crate::vehicle::input::VehicleInput;
use crate::vehicle::wheel::WheelAngleSpeed;

pub fn update_physics(
    time: Res<Time>,
    input: Res<VehicleInput>,
    params: Res<VehicleParams>,
    tire: Res<TireParams>,
    mut wheel_spd: ResMut<WheelAngleSpeed>,
    mut state: ResMut<VehicleState>,
) {
    let pacejka = Pacejka {
        params: tire.clone(),
    };
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

    state.delta_fl = (params.wheelbase * input.steering.tan()
        / (params.wheelbase - params.w / 2.0 * input.steering.tan()))
    .atan();
    state.delta_fr = (params.wheelbase * input.steering.tan()
        / (params.wheelbase + params.w / 2.0 * input.steering.tan()))
    .atan();

    let vx_fl_w = vx_fl * state.delta_fl.cos() + vy_fl * state.delta_fl.sin();
    let vy_fl_w = -vx_fl * state.delta_fl.sin() + vy_fl * state.delta_fl.cos();
    let vx_fr_w = vx_fr * state.delta_fr.cos() + vy_fr * state.delta_fr.sin();
    let vy_fr_w = -vx_fr * state.delta_fr.sin() + vy_fr * state.delta_fr.cos();

    let delta_fz_long = params.m * state.ax * params.h_cg / params.wheelbase;
    let delta_fz_lat = params.m * state.ay * params.h_cg / params.w;

    // Силы
    let (fx_fl, fy_fl) = pacejka.compute(
        safe_slip_angle(vy_fl_w, vx_fl_w),
        0.0,
        ((params.m * 9.81 * params.lr / (params.wheelbase * 2.0))
            - delta_fz_long / 2.0
            - delta_fz_lat / 2.0)
            .max(0.0),
    );
    let (fx_fr, fy_fr) = pacejka.compute(
        safe_slip_angle(vy_fr_w, vx_fr_w),
        0.0,
        ((params.m * 9.81 * params.lr / (params.wheelbase * 2.0)) - delta_fz_long / 2.0
            + delta_fz_lat / 2.0)
            .max(0.0),
    );
    let (fx_rl, fy_rl) = pacejka.compute(
        safe_slip_angle(vy_rl, vx_rl),
        0.0,
        ((params.m * 9.81 * params.lf / (params.wheelbase * 2.0)) + delta_fz_long / 2.0
            - delta_fz_lat / 2.0)
            .max(0.0),
    );
    let (fx_rr, fy_rr) = pacejka.compute(
        safe_slip_angle(vy_rr, vx_rr),
        0.0,
        ((params.m * 9.81 * params.lf / (params.wheelbase * 2.0))
            + delta_fz_long / 2.0
            + delta_fz_lat / 2.0)
            .max(0.0),
    );

    let fx_fl_body = (fdr / 4.0 + fx_fl) * (state.delta_fl * state.vx.signum()).cos()
        - fy_fl * (state.delta_fl * state.vx.signum()).sin();
    let fy_fl_body = (fdr / 4.0 + fx_fl) * (state.delta_fl * state.vx.signum()).sin()
        + fy_fl * (state.delta_fl * state.vx.signum()).cos();
    let fx_fr_body = (fdr / 4.0 + fx_fr) * (state.delta_fr * state.vx.signum()).cos()
        - fy_fr * (state.delta_fr * state.vx.signum()).sin();
    let fy_fr_body = (fdr / 4.0 + fx_fr) * (state.delta_fr * state.vx.signum()).sin()
        + fy_fr * (state.delta_fr * state.vx.signum()).cos();

    let fx_rl_body = fdr / 4.0 + fx_rl;
    let fy_rl_body = fy_rl;
    let fx_rr_body = fdr / 4.0 + fx_rr;
    let fy_rr_body = fy_rr;

    // Сопротивления
    let f_rolling = (params.c_rolling * params.m * 9.81) * state.vx.signum();
    let f_aero = 0.5 * params.rho_air * params.cd_a * state.vx * state.vx.abs();
    let f_drivetrain = params.c_drivetrain * state.vx;
    let omega_damp = params.c_omega_damp * state.omega;
    let vy_damp = params.c_vy_damp * state.vy;
    let total_resist = f_rolling + f_aero + f_drivetrain;

    let sum_fx = fx_fl_body + fx_fr_body + fx_rl_body + fx_rr_body - total_resist;
    let sum_fy = fy_fl_body + fy_fr_body + fy_rl_body + fy_rr_body - vy_damp;
    let sum_m = params.lf * (fy_fl_body + fy_fr_body) - params.lr * (fy_rl_body + fy_rr_body)
        + (params.w / 2.0) * (-fx_fl_body + fx_fr_body - fx_rl_body + fx_rr_body)
        - omega_damp;

    let vx_old = state.vx;
    let vy_old = state.vy;

    state.ax = sum_fx / params.m;
    state.ay = sum_fy / params.m;

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
