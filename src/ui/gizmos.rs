use bevy::{
    color::palettes::css::{BLUE, GREEN, RED},
    prelude::*,
};

use crate::{
    physics::state::{VehicleParams, VehicleState},
    ui::{forces::DebugForces, visibility::DebugVisibility},
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
