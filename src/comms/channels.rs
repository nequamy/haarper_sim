use bevy::prelude::*;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

use crossbeam_channel::{Receiver, Sender, bounded};

use crate::{
    comms::encoders::{bno055, rplidar, vesc},
    physics::{battery::BatteryState, motor::MotorState},
    sensors::{imu::ImuData, lidar::LidarScan},
    vehicle::input::VehicleInput,
};

pub struct CommandMessage {
    pub duty: f32,
    pub steering: f32,
}

#[derive(Resource)]
pub struct CommsChannels {
    pub imu_tx: Sender<Vec<u8>>,
    pub imu_rx: Receiver<Vec<u8>>,
    pub lidar_tx: Sender<Vec<u8>>,
    pub lidar_rx: Receiver<Vec<u8>>,
    pub vesc_tx: Sender<Vec<u8>>,
    pub vesc_rx: Receiver<Vec<u8>>,
    pub cmd_tx: Sender<CommandMessage>,
    pub cmd_rx: Receiver<CommandMessage>,
    pub imu_connected: Arc<AtomicBool>,
    pub lidar_connected: Arc<AtomicBool>,
    pub vesc_connected: Arc<AtomicBool>,
}

impl CommsChannels {
    pub fn new() -> Self {
        let (imu_tx, imu_rx) = bounded(64);
        let (lidar_tx, lidar_rx) = bounded(8);
        let (vesc_tx, vesc_rx) = bounded(64);
        let (cmd_tx, cmd_rx) = bounded(16);
        Self {
            imu_tx,
            imu_rx,
            lidar_tx,
            lidar_rx,
            vesc_tx,
            vesc_rx,
            cmd_tx,
            cmd_rx,
            imu_connected: Arc::new(AtomicBool::new(false)),
            lidar_connected: Arc::new(AtomicBool::new(false)),
            vesc_connected: Arc::new(AtomicBool::new(false)),
        }
    }
}

pub fn publish_imu(imu: Res<ImuData>, ch: Res<CommsChannels>) {
    if !ch.imu_connected.load(Ordering::Relaxed) {
        return;
    };
    if !imu.is_changed() {
        return;
    }
    let encoded = bno055::encode(imu.accel, imu.gyro, imu.quaternion);
    let _ = ch.imu_tx.try_send(encoded.to_vec());
}

pub fn publish_lidar(scan: Res<LidarScan>, ch: Res<CommsChannels>) {
    if !ch.lidar_connected.load(Ordering::Relaxed) {
        return;
    }
    if !scan.is_changed() {
        return;
    }
    let encoded = rplidar::encode(&scan);
    let _ = ch.lidar_tx.try_send(encoded);
}

pub fn publish_vesc(
    motor: Res<MotorState>,
    battery: Res<BatteryState>,
    ch: Res<CommsChannels>,
    time: Res<Time>,
    mut timer: Local<Option<Timer>>,
) {
    let t = timer.get_or_insert_with(|| Timer::from_seconds(0.02, TimerMode::Repeating));
    t.tick(time.delta());
    if !t.just_finished() {
        return;
    }
    if !ch.vesc_connected.load(Ordering::Relaxed) {
        return;
    }
    let encoded = vesc::encode_get_values(&motor, &battery);
    let _ = ch.vesc_tx.try_send(encoded);
}

pub fn apply_external_commands(ch: Res<CommsChannels>, mut input: ResMut<VehicleInput>) {
    let mut latest = None;
    while let Ok(cmd) = ch.cmd_rx.try_recv() {
        latest = Some(cmd);
    }
    if let Some(cmd) = latest {
        input.throttle = cmd.duty;
        input.steering = cmd.steering;
    }
}
