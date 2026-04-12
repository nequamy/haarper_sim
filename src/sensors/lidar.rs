use bevy::{color::palettes::css::RED, prelude::*};
use bevy_rapier3d::{plugin::ReadRapierContext, prelude::QueryFilter};
use rand::rng;
use rand_distr::{Distribution, Normal};

use crate::{
    physics::state::{Robot, VehicleState},
    ui::resources::DebugVisibility,
};

#[derive(Resource)]
pub struct LidarParams {
    pub num_rays: usize,
    pub max_range: f32,
    pub scan_hz: f32,
    pub noise_std: f32,
    pub offset: Vec2,
    pub timer: Timer,
}

impl Default for LidarParams {
    fn default() -> Self {
        Self {
            num_rays: 3200,
            max_range: 12.0,
            scan_hz: 10.0,
            noise_std: 0.005,
            offset: Vec2::new(-0.05, 0.0),
            timer: Timer::from_seconds(0.1, TimerMode::Repeating),
        }
    }
}

#[derive(Resource)]
pub struct LidarScan {
    pub ranges: Vec<f32>,
    pub angles: Vec<f32>,
    pub timestamps: f64,
}

impl LidarScan {
    pub fn new() -> Self {
        Self {
            ranges: vec![0.0; 3200],
            angles: (0..3200)
                .map(|i| (i as f32 / 3200.0) * std::f32::consts::TAU)
                .collect(),
            timestamps: 0.0,
        }
    }
}

pub fn simulate_scan(
    mut params: ResMut<LidarParams>,
    mut scan: ResMut<LidarScan>,
    time: Res<Time>,
    rapier: ReadRapierContext,
    state: Res<VehicleState>,
    entity: Query<Entity, With<Robot>>,
) {
    params.timer.tick(time.delta());
    if !params.timer.just_finished() {
        return;
    }

    let Ok(context) = rapier.single() else {
        return;
    };

    let Ok(robot_entity) = entity.single() else {
        return;
    };

    let forward = Vec3::new(state.yaw.cos(), 0.0, state.yaw.sin());

    let origin = Vec3::new(state.x, 0.25, state.y) + forward * (-0.05);

    let mut rng = rng();

    for i in 0..params.num_rays {
        let ray_angle = state.yaw + (i as f32 / params.num_rays as f32) * std::f32::consts::TAU;
        let direction = Vec3::new(ray_angle.cos(), 0.0, ray_angle.sin());

        let distance = match context.cast_ray(
            origin,
            direction,
            params.max_range,
            true,
            QueryFilter::default().exclude_rigid_body(robot_entity),
        ) {
            Some((_, dist)) => (dist * 1000.0).round() / 1000.0,
            None => params.max_range,
        };

        let noise_std_actual =
            Normal::new(0.0, params.noise_std * (1.0 + distance / params.max_range))
                .unwrap_or(Normal::new(0.0, 0.001).unwrap());
        scan.ranges[i] = (distance + noise_std_actual.sample(&mut rng) as f32).max(0.0);
    }
    scan.timestamps = time.elapsed_secs_f64();
}

pub fn draw_lidar_gizmos(
    mut gizmo: Gizmos,
    scan: Res<LidarScan>,
    params: Res<LidarParams>,
    state: Res<VehicleState>,
    debug: Res<DebugVisibility>,
) {
    if !debug.show_lidar_ray {
        return;
    }

    let forward = Vec3::new(state.yaw.cos(), 0.0, state.yaw.sin());

    let origin = Vec3::new(state.x, 0.25, state.y) + forward * (-0.05);

    for i in 0..params.num_rays {
        if scan.ranges[i] >= params.max_range {
            continue;
        }

        let direction = Vec3::new(
            (state.yaw + scan.angles[i]).cos(),
            0.0,
            (state.yaw + scan.angles[i]).sin(),
        );

        gizmo.line(origin, origin + direction * scan.ranges[i], RED);
    }
}
