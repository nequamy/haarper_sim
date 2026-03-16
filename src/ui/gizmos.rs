use bevy::{
    color::palettes::css::{BLUE, GRAY, GREEN, RED, YELLOW},
    math::ops::atan2,
    prelude::*,
};

use crate::{
    physics::state::{VehicleParams, VehicleState},
    ui::resources::{DebugForces, DebugVisibility, TrailHistory},
};

pub fn update_velocity_gizmos(
    mut gizmo: Gizmos,
    state: Res<VehicleState>,
    debug: Res<DebugVisibility>,
) {
    if !debug.show_velocity {
        return;
    }

    let forward = Vec3::new(state.yaw.cos(), 0.0, state.yaw.sin());
    let right = Vec3::new(-state.yaw.sin(), 0.0, state.yaw.cos());

    let start = Vec3::new(state.x, 0.25, state.y);

    gizmo.arrow(start, start + forward * state.vx * 0.5, RED);
    gizmo.arrow(start, start + right * state.vy * 0.5, GREEN);
}

pub fn update_forces_gizmos(
    mut gizmo: Gizmos,
    state: Res<VehicleState>,
    params: Res<VehicleParams>,
    forces: Res<DebugForces>,
    debug: Res<DebugVisibility>,
) {
    if !debug.show_tire_forces {
        return;
    }

    let scale = 1.0 / 40.0;

    let forward = Vec3::new(state.yaw.cos(), 0.0, state.yaw.sin());
    let right = Vec3::new(-state.yaw.sin(), 0.0, state.yaw.cos());
    let up = Vec3::Y;

    let cg = Vec3::new(state.x, 0.20, state.y);

    let offsets = [
        (params.lf, -params.w / 2.0),
        (params.lf, params.w / 2.0),
        (-params.lr, -params.w / 2.0),
        (-params.lr, params.w / 2.0),
    ];

    for (i, (fwd_off, side_off)) in offsets.iter().enumerate() {
        let pos = cg + forward * *fwd_off + right * *side_off;
        gizmo.arrow(pos, pos + forward * forces.fx[i] * scale, RED);
        gizmo.arrow(pos, pos + right * forces.fy[i] * scale, GREEN);
        gizmo.arrow(pos, pos + up * forces.fz[i] * scale, BLUE);
    }
}

pub fn record_trail(state: Res<VehicleState>, mut trail: ResMut<TrailHistory>) {
    let pos = Vec3::new(state.x, 0.05, state.y);

    if trail.trail.is_empty()
        || pos.distance(*trail.trail.back().unwrap_or(&Vec3::new(0.0, 0.0, 0.0))) > 0.05
    {
        trail.trail.push_back(pos);
    }

    if trail.trail.len() > 1000 {
        trail.trail.pop_front();
    }
}

pub fn update_trail_gizmos(
    mut gizmo: Gizmos,
    trail: Res<TrailHistory>,
    debug: Res<DebugVisibility>,
) {
    if !debug.show_trail {
        return;
    }

    gizmo.linestrip(trail.trail.iter().copied(), RED);
}

pub fn update_slip_angle(
    mut gizmo: Gizmos,
    state: Res<VehicleState>,
    params: Res<VehicleParams>,
    forces: Res<DebugForces>,
    debug: Res<DebugVisibility>,
) {
    if !debug.show_slip {
        return;
    }

    let forward = Vec3::new(state.yaw.cos(), 0.0, state.yaw.sin());
    let right = Vec3::new(-state.yaw.sin(), 0.0, state.yaw.cos());

    let cg = Vec3::new(state.x, 0.20, state.y);

    let offsets = [
        (params.lf, -params.w / 2.0, state.delta_fl),
        (params.lf, params.w / 2.0, state.delta_fr),
    ];

    for (i, (fwd_off, side_off, angle)) in offsets.iter().enumerate() {
        let pos = cg + forward * *fwd_off + right * *side_off;
        let wheel_dir = forward * angle.cos() + right * angle.sin();

        gizmo.arrow(pos, pos + wheel_dir * 0.5, YELLOW);

        let vel_dir = forward * forces.wheel_vx[i] + right * forces.wheel_vy[i];
        let vel_norm = vel_dir.normalize_or_zero();
        gizmo.arrow(pos, pos + vel_norm * 0.5, YELLOW);

        let heading_angle = atan2(wheel_dir.z, wheel_dir.x);
        let vel_angle = atan2(vel_norm.z, vel_norm.x);

        let mut diff = vel_angle - heading_angle;
        if diff > std::f32::consts::PI {
            diff -= std::f32::consts::TAU;
        }
        if diff < -std::f32::consts::PI {
            diff += std::f32::consts::TAU;
        }

        let arc_points: Vec<Vec3> = (0..=20)
            .map(|s| {
                let t = s as f32 / 20.0;
                let a = heading_angle + diff * t;
                pos + Vec3::new(a.cos(), 0.0, a.sin()) * 0.3
            })
            .collect();
        gizmo.linestrip(arc_points, YELLOW);
    }
}

pub fn update_slip_ratio(
    mut gizmo: Gizmos,
    state: Res<VehicleState>,
    params: Res<VehicleParams>,
    forces: Res<DebugForces>,
    debug: Res<DebugVisibility>,
) {
    if !debug.show_slip {
        return;
    }
    let forward = Vec3::new(state.yaw.cos(), 0.0, state.yaw.sin());
    let right = Vec3::new(-state.yaw.sin(), 0.0, state.yaw.cos());

    let cg = Vec3::new(state.x, 0.055, state.y);

    let offsets = [
        (params.lf, -params.w / 2.0 - 0.025, state.delta_fl),
        (params.lf, params.w / 2.0 + 0.025, state.delta_fr),
        (-params.lr + 0.010, -params.w / 2.0 - 0.025, 0.0),
        (-params.lr + 0.010, params.w / 2.0 + 0.025, 0.0),
    ];

    for (i, (fwd_off, side_off, angle)) in offsets.iter().enumerate() {
        let pos = cg + forward * *fwd_off + right * *side_off;
        let normal = -forward * angle.sin() + right * angle.cos();

        let rot = Quat::from_rotation_arc(Vec3::Z, normal.normalize());
        let iso = Isometry3d::new(pos, rot);

        gizmo.circle(iso, params.r_wheel, GRAY);

        let fill = forces.kappa[i].abs().min(1.0);
        let steps = 100;
        let filled = (fill * steps as f32) as usize;
        for j in 1..=filled {
            let r = params.r_wheel * (j as f32 / steps as f32);
            gizmo.circle(iso, r, Color::srgb(fill, 1.0 - fill, 0.0));
        }
    }
}
