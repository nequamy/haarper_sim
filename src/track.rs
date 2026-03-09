use bevy::prelude::*;
use bevy_rapier3d::prelude::{Collider, RigidBody};

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_track);
    }
}

fn spawn_track(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let track_data = load_centerline("assets/tracks/sochi/Sochi_centerline.csv");
    let wall_height = 0.3;
    let wall_thickness = 0.02;

    let wall_mat = materials.add(Color::srgb(0.5, 0.5, 0.5));
    let road_mat = materials.add(Color::srgb(0.15, 0.15, 0.15));
    let dash_mat = materials.add(Color::srgb(0.9, 0.9, 0.9));

    let n = track_data.len();

    let mut left_points = Vec::with_capacity(n);
    let mut right_points = Vec::with_capacity(n);

    for i in 0..n {
        let prev = track_data[(i + n - 1) % n].0;
        let curr = track_data[i].0;
        let next = track_data[(i + 1) % n].0;

        let dir1 = (curr - prev).normalize();
        let dir2 = (next - curr).normalize();
        let avg_dir = (dir1 + dir2).normalize();
        let normal = Vec2::new(-avg_dir.y, avg_dir.x);

        let (_, w_right, w_left) = track_data[i];
        left_points.push(curr + normal * w_left);
        right_points.push(curr - normal * w_right);
    }

    for i in 0..n {
        let next = (i + 1) % n;

        let lp0 = left_points[i];
        let lp1 = left_points[next];
        let l_center = (lp0 + lp1) / 2.0;
        let l_dir = (lp1 - lp0).normalize();
        let l_length = (lp1 - lp0).length() + wall_thickness;
        let l_angle = l_dir.y.atan2(l_dir.x);

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(l_length, wall_height, wall_thickness))),
            MeshMaterial3d(wall_mat.clone()),
            Transform::from_xyz(l_center.x, wall_height / 2.0, l_center.y)
                .with_rotation(Quat::from_rotation_y(-l_angle)),
            Collider::cuboid(l_length / 2.0, wall_height / 2.0, wall_thickness / 2.0),
            RigidBody::Fixed,
        ));

        let rp0 = right_points[i];
        let rp1 = right_points[next];
        let r_center = (rp0 + rp1) / 2.0;
        let r_dir = (rp1 - rp0).normalize();
        let r_length = (rp1 - rp0).length() + wall_thickness;
        let r_angle = r_dir.y.atan2(r_dir.x);

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(r_length, wall_height, wall_thickness))),
            MeshMaterial3d(wall_mat.clone()),
            Transform::from_xyz(r_center.x, wall_height / 2.0, r_center.y)
                .with_rotation(Quat::from_rotation_y(-r_angle)),
            Collider::cuboid(r_length / 2.0, wall_height / 2.0, wall_thickness / 2.0),
            RigidBody::Fixed,
        ));

        let road_mid_i = (left_points[i] + right_points[i]) / 2.0;
        let road_mid_next = (left_points[next] + right_points[next]) / 2.0;
        let road_center = (road_mid_i + road_mid_next) / 2.0;
        let road_dir = (road_mid_next - road_mid_i).normalize();
        let road_length = (road_mid_next - road_mid_i).length() * 1.3;
        let road_angle = road_dir.y.atan2(road_dir.x);

        let width_i = (left_points[i] - right_points[i]).length();
        let width_next = (left_points[next] - right_points[next]).length();
        let road_width = (width_i + width_next) / 2.0 + 0.2;

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(road_length, 0.005, road_width))),
            MeshMaterial3d(road_mat.clone()),
            Transform::from_xyz(road_center.x, 0.001, road_center.y)
                .with_rotation(Quat::from_rotation_y(-road_angle)),
        ));

        if i % 2 == 0 {
            let pos = track_data[i].0;
            let pos_next = track_data[next].0;
            let center = (pos + pos_next) / 2.0;
            let dir = (pos_next - pos).normalize();
            let length = (pos_next - pos).length() + wall_thickness;
            let angle = dir.y.atan2(dir.x);

            commands.spawn((
                Mesh3d(meshes.add(Cuboid::new(length * 0.6, 0.006, 0.03))),
                MeshMaterial3d(dash_mat.clone()),
                Transform::from_xyz(center.x, 0.002, center.y)
                    .with_rotation(Quat::from_rotation_y(-angle)),
            ));
        }
    }
}

fn oval(cx: f32, cy: f32, rx: f32, ry: f32, n: usize) -> Vec<Vec2> {
    (0..n)
        .map(|i| {
            let t = (i as f32 / n as f32) * std::f32::consts::TAU;
            Vec2::new(cx + rx * t.cos(), cy + ry * t.sin())
        })
        .collect()
}

fn load_centerline(path: &str) -> Vec<(Vec2, f32, f32)> {
    let content = std::fs::read_to_string(path).expect("Failed to read track CSV");
    let mut data: Vec<(Vec2, f32, f32)> = content
        .lines()
        .filter(|line| !line.starts_with('#') && !line.is_empty())
        .map(|line| {
            let vals: Vec<f32> = line.split(',').map(|s| s.trim().parse().unwrap()).collect();
            (Vec2::new(vals[0], vals[1]), vals[2], vals[3])
        })
        .collect();

    let center = data.iter().map(|(p, _, _)| *p).sum::<Vec2>() / data.len() as f32;
    for (pos, _, _) in &mut data {
        *pos -= center;
    }
    data
}
