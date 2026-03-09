use crate::physics::state::Robot;
use crate::vehicle::wheel::{Wheel, WheelF};
use bevy::prelude::*;
use bevy_rapier3d::prelude::{
    CharacterLength, Collider, CollisionGroups, Group, KinematicCharacterController, RigidBody,
};

pub fn spawn_vehicle(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands
        .spawn((
            SceneRoot(asset_server.load("chassis.glb#Scene0")),
            Transform::from_xyz(10.0, 0.095, 10.0),
            Collider::cuboid(0.38 / 2.0, 0.25 / 2.0, 0.25 / 2.0),
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
                WheelF,
            ));

            parent.spawn((
                SceneRoot(asset_server.load("wheel.glb#Scene0")),
                Transform::from_translation(Vec3::new(0.18, -0.040, 0.125))
                    .with_rotation(wheel_rot),
                WheelF,
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
