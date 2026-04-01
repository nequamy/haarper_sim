use bevy::anti_alias::taa::TemporalAntiAliasing;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::pbr::DistanceFog;
use bevy::post_process::dof::DepthOfField;
use bevy::post_process::motion_blur::MotionBlur;
use bevy::prelude::*;
use bevy_panorbit_camera::PanOrbitCamera;
use bevy_panorbit_camera::PanOrbitCameraPlugin;

use crate::SimState;
use crate::physics::state::VehicleState;

#[derive(Resource, Default)]
struct CameraMode {
    follow: bool,
}

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraMode>()
            .add_plugins(PanOrbitCameraPlugin)
            .add_systems(Startup, spawn_camera)
            .add_systems(Update, camera_follow.run_if(in_state(SimState::Running)));
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        PanOrbitCamera::default(),
        Msaa::Off,
        Transform::from_xyz(0.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        Tonemapping::TonyMcMapface,
        MotionBlur {
            shutter_angle: 0.5,
            samples: 2,
        },
        DepthOfField {
            focal_distance: 5.0,
            aperture_f_stops: 8.0,
            max_depth: 100.0,
            ..default()
        },
        TemporalAntiAliasing::default(),
        DistanceFog {
            color: Color::srgba(0.35, 0.48, 0.66, 1.0),
            directional_light_color: Color::srgba(1.0, 0.95, 0.85, 0.5),
            directional_light_exponent: 30.0,
            falloff: FogFalloff::Exponential { density: 0.01 },
        },
    ));
}

fn camera_follow(
    keys: Res<ButtonInput<KeyCode>>,
    mut mode: ResMut<CameraMode>,
    state: Res<VehicleState>,
    mut query: Query<&mut PanOrbitCamera>,
) {
    if keys.just_pressed(KeyCode::Tab) {
        mode.follow = !mode.follow;
    }

    if mode.follow {
        let Ok(mut cam) = query.single_mut() else {
            return;
        };
        cam.target_focus = Vec3::new(state.x, 0.0, state.y);
        cam.target_yaw = -state.yaw - std::f32::consts::FRAC_PI_2;
        cam.target_pitch = 0.3;
        cam.target_radius = 3.0;
    }
}
