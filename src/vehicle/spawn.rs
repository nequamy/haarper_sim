use crate::physics::state::{Robot, VehicleState};
use crate::track::TrackData;
use crate::vehicle::wheel::{Wheel, WheelFL, WheelFR};
use bevy::math::ops::atan2;
use bevy::prelude::*;
use bevy_rapier3d::prelude::{
    CharacterLength, Collider, CollisionGroups, Group, KinematicCharacterController, RigidBody,
};

pub fn spawn_vehicle(
    track: Res<TrackData>,
    asset_server: Res<AssetServer>,
    mut commands: Commands,
    mut state: ResMut<VehicleState>,
) {
    let (x, y, z) = get_start_position(&track);
    let yaw = get_start_heading(&track);

    state.x = x;
    state.y = z;
    state.yaw = yaw;

    commands
        .spawn((
            SceneRoot(asset_server.load("chassis.glb#Scene0")),
            Transform::from_xyz(x, y, z).with_rotation(Quat::from_rotation_y(-yaw)),
            Collider::cuboid(0.55 / 2.0, 0.10 / 2.0, 0.21 / 2.0),
            RigidBody::KinematicPositionBased,
            KinematicCharacterController {
                snap_to_ground: None,
                offset: CharacterLength::Absolute(0.01),
                ..default()
            },
            CollisionGroups::new(Group::GROUP_2, Group::GROUP_1),
            Robot,
        ))
        .with_children(|parent| {
            let wheel_rot = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);

            parent.spawn((
                SceneRoot(asset_server.load("wheel.glb#Scene0")),
                Transform::from_translation(Vec3::new(-0.14, -0.040, -0.125))
                    .with_rotation(wheel_rot),
                Wheel,
            ));

            parent.spawn((
                SceneRoot(asset_server.load("wheel.glb#Scene0")),
                Transform::from_translation(Vec3::new(0.18, -0.040, -0.125))
                    .with_rotation(wheel_rot),
                WheelFR,
            ));

            parent.spawn((
                SceneRoot(asset_server.load("wheel.glb#Scene0")),
                Transform::from_translation(Vec3::new(0.18, -0.040, 0.125))
                    .with_rotation(wheel_rot),
                WheelFL,
            ));

            parent.spawn((
                SceneRoot(asset_server.load("wheel.glb#Scene0")),
                Transform::from_translation(Vec3::new(-0.14, -0.040, 0.125))
                    .with_rotation(wheel_rot),
                Wheel,
            ));

            parent.spawn((
                SceneRoot(asset_server.load("lidar.glb#Scene0")),
                Transform::from_translation(Vec3::new(-0.05, 0.145, 0.0)),
            ));
        });
}

pub fn get_start_position(track: &TrackData) -> (f32, f32, f32) {
    (track.data[0].0.x, 0.095, track.data[0].0.y)
}

pub fn get_start_heading(track: &TrackData) -> f32 {
    if track.data.len() < 10 {
        return std::f32::consts::TAU;
    }

    atan2(
        track.data[10].0.y - track.data[0].0.y,
        track.data[10].0.x - track.data[0].0.x,
    )
}
