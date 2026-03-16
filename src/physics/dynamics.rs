use bevy::math::FloatPow;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::physics::battery::{BatteryParams, BatteryState};
use crate::physics::motor::{MotorParams, MotorState};
use crate::physics::servo::{ServoParams, ServoState};
use crate::physics::state::{Robot, VehicleParams, VehicleState};
use crate::physics::tire_model::{Pacejka, TireParams};
use crate::ui::resources::DebugForces;
use crate::vehicle::input::VehicleInput;
use crate::vehicle::wheel::{WheelAngleSpeed, WheelDynamics};

pub fn update_physics(
    time: Res<Time>,
    input: Res<VehicleInput>,
    params: Res<VehicleParams>,
    tire: Res<TireParams>,
    battery_params: Res<BatteryParams>,
    motor_params: Res<MotorParams>,
    servo_params: Res<ServoParams>,
    mut battery: ResMut<BatteryState>,
    mut motor: ResMut<MotorState>,
    mut wheel_spd: ResMut<WheelAngleSpeed>,
    mut wheel_dynamics: ResMut<WheelDynamics>,
    mut state: ResMut<VehicleState>,
    mut servo: ResMut<ServoState>,
    mut debug_forces: ResMut<DebugForces>,
) {
    let pacejka = Pacejka {
        params: tire.clone(),
    };

    let dt = time.delta_secs();

    let duty = input.throttle;
    let t_motor = motor.compute(
        duty,
        (wheel_dynamics.omega_fl
            + wheel_dynamics.omega_fr
            + wheel_dynamics.omega_rl
            + wheel_dynamics.omega_rr)
            / 4.0,
        battery.v_terminal,
        &motor_params,
    );
    battery.update(motor.current.abs(), dt, &battery_params);

    let t_per_wheel = t_motor * motor_params.gear_ratio * motor_params.eta / 4.0;

    wheel_spd.angle += (state.vx / params.r_wheel) * dt;
    wheel_spd.angle = wheel_spd.angle.rem_euclid(std::f32::consts::TAU);

    // Скорости
    let vx_fl = state.vx - (params.w / 2.0) * state.omega;
    let vy_fl = state.vy + params.lf * state.omega;
    let vx_fr = state.vx + (params.w / 2.0) * state.omega;
    let vy_fr = state.vy + params.lf * state.omega;
    let vx_rl = state.vx - (params.w / 2.0) * state.omega;
    let vy_rl = state.vy - params.lr * state.omega;
    let vx_rr = state.vx + (params.w / 2.0) * state.omega;
    let vy_rr = state.vy - params.lr * state.omega;

    servo.update(input.steering, dt, &servo_params);

    state.delta_fl = (params.wheelbase * servo.delta.tan()
        / (params.wheelbase - params.w / 2.0 * servo.delta.tan()))
    .atan();
    state.delta_fr = (params.wheelbase * servo.delta.tan()
        / (params.wheelbase + params.w / 2.0 * servo.delta.tan()))
    .atan();

    let vx_fl_w = vx_fl * state.delta_fl.cos() + vy_fl * state.delta_fl.sin();
    let vy_fl_w = -vx_fl * state.delta_fl.sin() + vy_fl * state.delta_fl.cos();
    let vx_fr_w = vx_fr * state.delta_fr.cos() + vy_fr * state.delta_fr.sin();
    let vy_fr_w = -vx_fr * state.delta_fr.sin() + vy_fr * state.delta_fr.cos();

    debug_forces.wheel_vx = [vx_fl_w, vx_fr_w, vx_rl, vx_rr];
    debug_forces.wheel_vy = [vy_fl_w, vy_fr_w, vy_rl, vy_rr];

    let delta_fz_long = params.m * state.ax * params.h_cg / params.wheelbase;
    let delta_fz_lat = params.m * state.ay * params.h_cg / params.w;

    // Силы
    let fz_fl = ((params.m * 9.81 * params.lr / (params.wheelbase * 2.0))
        - delta_fz_long / 2.0
        - delta_fz_lat / 2.0)
        .max(0.0);
    let fz_fr = ((params.m * 9.81 * params.lr / (params.wheelbase * 2.0)) - delta_fz_long / 2.0
        + delta_fz_lat / 2.0)
        .max(0.0);
    let fz_rl = ((params.m * 9.81 * params.lf / (params.wheelbase * 2.0)) + delta_fz_long / 2.0
        - delta_fz_lat / 2.0)
        .max(0.0);
    let fz_rr = ((params.m * 9.81 * params.lf / (params.wheelbase * 2.0))
        + delta_fz_long / 2.0
        + delta_fz_lat / 2.0)
        .max(0.0);

    let kappa_fl = (wheel_dynamics.omega_fl * params.r_wheel - vx_fl_w)
        / (wheel_dynamics.omega_fl * params.r_wheel)
            .abs()
            .max(vx_fl_w.abs())
            .max(0.01);
    let kappa_fr = (wheel_dynamics.omega_fr * params.r_wheel - vx_fr_w)
        / (wheel_dynamics.omega_fr * params.r_wheel)
            .abs()
            .max(vx_fr_w.abs())
            .max(0.01);
    let kappa_rl = (wheel_dynamics.omega_rl * params.r_wheel - vx_rl)
        / (wheel_dynamics.omega_rl * params.r_wheel)
            .abs()
            .max(vx_rl.abs())
            .max(0.01);
    let kappa_rr = (wheel_dynamics.omega_rr * params.r_wheel - vx_rr)
        / (wheel_dynamics.omega_rr * params.r_wheel)
            .abs()
            .max(vx_rr.abs())
            .max(0.01);

    let (fx_fl, fy_fl) = pacejka.compute(safe_slip_angle(vy_fl_w, vx_fl_w), kappa_fl, fz_fl);
    let (fx_fr, fy_fr) = pacejka.compute(safe_slip_angle(vy_fr_w, vx_fr_w), kappa_fr, fz_fr);
    let (fx_rl, fy_rl) = pacejka.compute(safe_slip_angle(vy_rl, vx_rl), kappa_rl, fz_rl);
    let (fx_rr, fy_rr) = pacejka.compute(safe_slip_angle(vy_rr, vx_rr), kappa_rr, fz_rr);

    debug_forces.fx = [fx_fl, fx_fr, fx_rl, fx_rr];
    debug_forces.fy = [fy_fl, fy_fr, fy_rl, fy_rr];
    debug_forces.fz = [fz_fl, fz_fr, fz_rl, fz_rr];
    debug_forces.kappa = [kappa_fl, kappa_fr, kappa_rl, kappa_rr];
    debug_forces.alpha = [
        safe_slip_angle(vy_fl_w, vx_fl_w),
        safe_slip_angle(vy_fr_w, vx_fr_w),
        safe_slip_angle(vy_rl, vx_rl),
        safe_slip_angle(vy_rr, vx_rr),
    ];

    wheel_dynamics.omega_fl += (t_per_wheel
        - fx_fl * params.r_wheel
        - (wheel_dynamics.omega_fl * 0.00005)
        - (0.0000014 * motor_params.gear_ratio.squared() * wheel_dynamics.omega_fl))
        / wheel_dynamics.j_eff
        * dt;
    wheel_dynamics.omega_fr += (t_per_wheel
        - fx_fr * params.r_wheel
        - (wheel_dynamics.omega_fr * 0.00005)
        - (0.0000014 * motor_params.gear_ratio.squared() * wheel_dynamics.omega_fr))
        / wheel_dynamics.j_eff
        * dt;
    wheel_dynamics.omega_rl += (t_per_wheel
        - fx_rl * params.r_wheel
        - (wheel_dynamics.omega_rl * 0.00005)
        - (0.0000014 * motor_params.gear_ratio.squared() * wheel_dynamics.omega_rl))
        / wheel_dynamics.j_eff
        * dt;
    wheel_dynamics.omega_rr += (t_per_wheel
        - fx_rr * params.r_wheel
        - (wheel_dynamics.omega_rr * 0.00005)
        - (0.0000014 * motor_params.gear_ratio.squared() * wheel_dynamics.omega_rr))
        / wheel_dynamics.j_eff
        * dt;

    let fx_fl_body = fx_fl * state.delta_fl.cos() - fy_fl * state.delta_fl.sin();
    let fy_fl_body = fx_fl * state.delta_fl.sin() + fy_fl * state.delta_fl.cos();
    let fx_fr_body = fx_fr * state.delta_fr.cos() - fy_fr * state.delta_fr.sin();
    let fy_fr_body = fx_fr * state.delta_fr.sin() + fy_fr * state.delta_fr.cos();

    let fx_rl_body = fx_rl;
    let fy_rl_body = fy_rl;
    let fx_rr_body = fx_rr;
    let fy_rr_body = fy_rr;

    // Сопротивления
    let f_rolling = (params.c_rolling * params.m * 9.81) * (state.vx / 0.05).tanh();
    let f_aero = 0.5 * params.rho_air * params.cd_a * state.vx * state.vx.abs();
    let omega_damp = params.c_omega_damp * state.omega;
    let vy_damp = params.c_vy_damp * state.vy;
    let total_resist = f_rolling + f_aero;

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
        -(vy).atan2(vx.abs())
    }
}
