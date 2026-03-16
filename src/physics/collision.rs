use bevy::prelude::*;
use bevy_rapier3d::prelude::KinematicCharacterControllerOutput;

use crate::{
    physics::state::{Robot, VehicleState},
    vehicle::wheel::WheelDynamics,
};

pub fn sync_vehicle_after_collision(
    mut state: ResMut<VehicleState>,
    mut wheel: ResMut<WheelDynamics>,
    query: Query<&Transform, With<Robot>>,
    controller: Query<&KinematicCharacterControllerOutput, With<Robot>>,
) {
    let Ok(tf) = query.single() else {
        return;
    };
    let Ok(coll) = controller.single() else {
        return;
    };

    let diff_x = coll.desired_translation.x - coll.effective_translation.x;
    let diff_y = coll.desired_translation.z - coll.effective_translation.z;

    if diff_x.abs() >= 0.001 || diff_y.abs() >= 0.001 {
        state.x = tf.translation.x;
        state.y = tf.translation.z;

        let mut wx = state.vx * state.yaw.cos() - state.vy * state.yaw.sin();
        let mut wy = state.vx * state.yaw.sin() + state.vy * state.yaw.cos();

        if coll.collisions.is_empty() {
            return;
        }

        let Some(details) = coll.collisions[0].hit.details else {
            return;
        };

        let mut nx = details.normal2.x;
        let mut nz = details.normal2.z;

        let len = (nx * nx + nz * nz).sqrt();
        if len > 0.001 {
            nx /= len;
            nz /= len;
        }

        let v_norm = wx * nx + wy * nz;
        if v_norm < 0.0 {
            return;
        }

        wx -= 1.15 * v_norm * nx;
        wy -= 1.15 * v_norm * nz;

        state.vx = wx * state.yaw.cos() + wy * state.yaw.sin();
        state.vy = -wx * state.yaw.sin() + wy * state.yaw.cos();

        wheel.kappa = [0.0; 4];
        wheel.alpha = [0.0; 4];
    }
}
