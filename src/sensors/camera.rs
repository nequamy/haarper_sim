use bevy::{camera::RenderTarget, prelude::*, render::render_resource::TextureFormat};

use crate::{physics::state::Robot, ui::resources::DebugVisibility};

#[derive(Resource, Default)]
pub struct RobotCameraImage {
    pub handle: Handle<Image>,
}

pub fn spawn_robot_camera(
    robot: Query<Entity, With<Robot>>,
    mut commands: Commands,
    mut image: ResMut<Assets<Image>>,
    mut camera_image: ResMut<RobotCameraImage>,
) {
    let Ok(robot_entity) = robot.single() else {
        return;
    };

    let image_texture = Image::new_target_texture(
        320,
        240,
        TextureFormat::Rgba8Unorm,
        Some(TextureFormat::Rgba8UnormSrgb),
    );

    let handle = image.add(image_texture);
    camera_image.handle = handle.clone();

    commands.entity(robot_entity).with_children(|parent| {
        parent.spawn((
            Camera3d::default(),
            Camera {
                order: -1,
                clear_color: Color::BLACK.into(),
                ..default()
            },
            RenderTarget::Image(handle.into()),
            Transform::from_xyz(0.1, 0.08, 0.0).looking_at(Vec3::X, Vec3::Y),
        ));
    });
}
