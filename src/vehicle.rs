use bevy::prelude::*;
use bevy_rapier3d::prelude::{
    CharacterLength, Collider, CollisionGroups, Group, KinematicCharacterController,
    KinematicCharacterControllerOutput, RigidBody, ShapeCastHitDetails,
};

#[derive(Resource)]
pub struct VehicleState {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
        }
    }
}

#[derive(Resource)]
pub struct VehicleInput {
    pub steering: f32,
    pub throttle: f32,
}

impl Default for VehicleInput {
    fn default() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
        }
    }
}

#[derive(Resource)]
pub struct WheelAngleSpeed {
    angle: f32,
}

impl Default for WheelAngleSpeed {
    fn default() -> Self {
        Self { angle: 0.0 }
    }
}

#[derive(Resource)]
pub struct VehicleParams {
    pub m: f32,
    pub iz: f32,
    pub lf: f32,
    pub lr: f32,
    pub wheelbase: f32,
    pub w: f32,
    pub r_wheel: f32,
    pub f_max: f32,
    pub tire_b: f32,
    pub tire_c: f32,
    pub tire_mu: f32,
}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            m: 4.0,
            iz: 0.05,
            lf: 0.18,
            lr: 0.15,
            wheelbase: 0.33,
            w: 0.25,
            r_wheel: 0.055,
            f_max: 11.5,
            tire_b: 13.0,
            tire_c: 1.3,
            tire_mu: 1.2,
        }
    }
}

#[derive(Component)]
struct Robot;

#[derive(Component)]
struct Wheel;

#[derive(Component)]
struct WheelF;

pub struct VehiclePlugin;

impl Plugin for VehiclePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_vehicle)
            .insert_resource(VehicleState::default())
            .insert_resource(VehicleInput::default())
            .insert_resource(VehicleParams::default())
            .insert_resource(WheelAngleSpeed::default())
            .insert_resource(Time::<Fixed>::from_hz(200.0))
            .add_systems(Update, read_input)
            .add_systems(Update, update_wheel_angle)
            .add_systems(Update, update_forward_wheel_angle)
            .add_systems(
                FixedUpdate,
                (update_physics, sync_vehicle_transform).chain(),
            )
            .add_systems(PostUpdate, sync_vehicle_after_collision);
    }
}

fn spawn_vehicle(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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

fn read_input(time: Res<Time>, keys: Res<ButtonInput<KeyCode>>, mut input: ResMut<VehicleInput>) {
    if keys.pressed(KeyCode::ShiftLeft) {
        return;
    }

    let dt = time.delta_secs();
    let throttle_rate = 3.0;
    let steering_rate = 2.0;

    let target_throttle = if keys.pressed(KeyCode::KeyW) {
        1.0
    } else if keys.pressed(KeyCode::KeyS) {
        -1.0
    } else {
        0.0
    };

    let target_steering = if keys.pressed(KeyCode::KeyA) {
        -0.2
    } else if keys.pressed(KeyCode::KeyD) {
        0.2
    } else {
        0.0
    };

    let diff_t = target_throttle - input.throttle;
    input.throttle += diff_t.clamp(-throttle_rate * dt, throttle_rate * dt);

    let diff_s = target_steering - input.steering;
    input.steering += diff_s.clamp(-steering_rate * dt, steering_rate * dt);
}

fn update_physics(
    time: Res<Time>,
    input: Res<VehicleInput>,
    params: Res<VehicleParams>,
    mut wheel_spd: ResMut<WheelAngleSpeed>,
    mut state: ResMut<VehicleState>,
) {
    let dt = time.delta_secs();

    let fdr: f32 = input.throttle * params.f_max;

    wheel_spd.angle += (state.vx / params.r_wheel) * dt;

    // Скорости
    let vx_fl = state.vx - (params.w / 2.0) * state.omega;
    let vy_fl = state.vy + params.lf * state.omega;
    let vx_fr = state.vx + (params.w / 2.0) * state.omega;
    let vy_fr = state.vy + params.lf * state.omega;
    let vx_rl = state.vx - (params.w / 2.0) * state.omega;
    let vy_rl = state.vy - params.lr * state.omega;
    let vx_rr = state.vx + (params.w / 2.0) * state.omega;
    let vy_rr = state.vy - params.lr * state.omega;

    // Slip angle
    let vx_fl_w = vx_fl * input.steering.cos() + vy_fl * input.steering.sin();
    let vy_fl_w = -vx_fl * input.steering.sin() + vy_fl * input.steering.cos();
    let vx_fr_w = vx_fr * input.steering.cos() + vy_fr * input.steering.sin();
    let vy_fr_w = -vx_fr * input.steering.sin() + vy_fr * input.steering.cos();

    let alpha_fl = safe_slip_angle(vy_fl_w, vx_fl_w);
    let alpha_fr = safe_slip_angle(vy_fr_w, vx_fr_w);
    let alpha_rl = safe_slip_angle(vy_rl, vx_rl);
    let alpha_rr = safe_slip_angle(vy_rr, vx_rr);

    // Силы
    let ff_ly: f32 = (params.tire_mu * (params.m * 9.81 * params.lr / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_fl).atan()).sin();
    let ff_ry: f32 = (params.tire_mu * (params.m * 9.81 * params.lr / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_fr).atan()).sin();
    let fr_ly: f32 = (params.tire_mu * (params.m * 9.81 * params.lf / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_rl).atan()).sin();
    let fr_ry: f32 = (params.tire_mu * (params.m * 9.81 * params.lf / (params.wheelbase * 2.0)))
        * (params.tire_c * (params.tire_b * alpha_rr).atan()).sin();

    let fx_fl_body = (fdr / 4.0) * input.steering.cos() - ff_ly * input.steering.sin();
    let fy_fl_body = (fdr / 4.0) * input.steering.sin() + ff_ly * input.steering.cos();
    let fx_fr_body = (fdr / 4.0) * input.steering.cos() - ff_ry * input.steering.sin();
    let fy_fr_body = (fdr / 4.0) * input.steering.sin() + ff_ry * input.steering.cos();

    let fx_rl_body = fdr / 4.0;
    let fy_rl_body = fr_ly;
    let fx_rr_body = fdr / 4.0;
    let fy_rr_body = fr_ry;

    // Сопротивления
    let f_rolling = (0.015 * params.m * 9.81) * state.vx.signum();
    let f_aero = 0.5 * 1.225 * 0.4 * 0.02 * state.vx * state.vx.abs();
    let f_drivetrain = 1.0 * state.vx;
    let omega_damp = 0.04 * state.omega;
    let vy_damp = 1.0 * state.vy;
    let total_resist = f_rolling + f_aero + f_drivetrain;

    let sum_fx = fx_fl_body + fx_fr_body + fx_rl_body + fx_rr_body - total_resist;
    let sum_fy = fy_fl_body + fy_fr_body + fy_rl_body + fy_rr_body - vy_damp;
    let sum_m = params.lf * (fy_fl_body + fy_fr_body) - params.lr * (fy_rl_body + fy_rr_body)
        + (params.w / 2.0) * (-fx_fl_body + fx_fr_body - fx_rl_body + fx_rr_body)
        - omega_damp;

    let vx_old = state.vx;
    let vy_old = state.vy;

    state.vx += ((1.0 / params.m) * (sum_fx + params.m * vy_old * state.omega)) * dt;
    state.vy += ((1.0 / params.m) * (sum_fy - params.m * vx_old * state.omega)) * dt;
    state.omega += (1.0 / params.iz) * sum_m * dt;

    state.x += (state.vx * state.yaw.cos() - state.vy * state.yaw.sin()) * dt;
    state.y += (state.vx * state.yaw.sin() + state.vy * state.yaw.cos()) * dt;
    state.yaw += state.omega * dt;
}

fn safe_slip_angle(vy: f32, vx: f32) -> f32 {
    if vx.abs() < 0.5 {
        -(vy * 2.0).clamp(-1.0, 1.0)
    } else {
        -(vy / vx).atan()
    }
}

fn sync_vehicle_transform(
    state: Res<VehicleState>,
    mut query: Query<(&mut Transform, &mut KinematicCharacterController), With<Robot>>,
) {
    let Ok((mut tf, mut controller)) = query.single_mut() else {
        return;
    };

    controller.translation = Some(Vec3::new(
        state.x - tf.translation.x,
        0.0,
        state.y - tf.translation.z,
    ));

    // tf.translation.x = state.x;
    // tf.translation.z = state.y;
    tf.rotation = Quat::from_rotation_y(-state.yaw);
}

fn sync_vehicle_after_collision(
    mut state: ResMut<VehicleState>,
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
    }
}

fn update_forward_wheel_angle(
    input: Res<VehicleInput>,
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<WheelF>>,
) {
    let _ = query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(-input.steering + std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}

fn update_wheel_angle(
    wheel_spd: Res<WheelAngleSpeed>,
    mut query: Query<&mut Transform, With<Wheel>>,
) {
    let _ = query.iter_mut().for_each(|mut w| {
        w.rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
            * Quat::from_rotation_x(wheel_spd.angle)
    });
}
