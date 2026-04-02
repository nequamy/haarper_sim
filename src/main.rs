use bevy::prelude::*;

mod physics;
mod ui;
mod vehicle;

mod camera;
mod track;

use bevy_rapier3d::plugin::RapierPhysicsPlugin;
use camera::CameraPlugin;
use physics::PhysicsPlugin;
use track::TrackPlugin;
use ui::DebugUIPlugin;
use vehicle::VehiclePlugin;

use crate::physics::state::GameEntity;

#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimState {
    #[default]
    Menu,
    Running,
}

fn main() {
    App::new()
        .add_plugins(RapierPhysicsPlugin::<()>::default())
        .add_plugins(DefaultPlugins)
        .add_plugins(CameraPlugin)
        .add_plugins(VehiclePlugin)
        .add_plugins(PhysicsPlugin)
        .add_plugins(DebugUIPlugin)
        .add_plugins(TrackPlugin)
        .init_state::<SimState>()
        .add_systems(OnExit(SimState::Running), cleanup_sim)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(180.0, 180.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.55, 0.15),
            perceptual_roughness: 0.95,
            reflectance: 0.1,
            ..default()
        })),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, 0.2, 0.0)),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: false,
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, 1.5, 0.0)),
    ));
}

fn cleanup_sim(mut commands: Commands, query: Query<Entity, With<GameEntity>>) {
    for entity in &query {
        commands.entity(entity).despawn();
    }
}
