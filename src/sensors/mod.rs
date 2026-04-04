use bevy::prelude::*;

use crate::{
    SimState,
    sensors::{
        camera::{RobotCameraImage, spawn_robot_camera},
        imu::{ImuBias, ImuData, ImuParams, simulate_imu},
        lidar::{LidarParams, LidarScan, draw_lidar_gizmos, simulate_scan},
    },
    vehicle::spawn::spawn_vehicle,
};

pub mod camera;
pub mod imu;
pub mod lidar;

pub struct SensorPlugin;

impl Plugin for SensorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(LidarParams::default())
            .insert_resource(LidarScan::new())
            .add_systems(Update, simulate_scan.run_if(in_state(SimState::Running)))
            .add_systems(
                Update,
                draw_lidar_gizmos.run_if(in_state(SimState::Running)),
            )
            .insert_resource(ImuBias::default())
            .insert_resource(ImuData::default())
            .insert_resource(ImuParams::default())
            .add_systems(Update, simulate_imu.run_if(in_state(SimState::Running)))
            .insert_resource(RobotCameraImage::default())
            .add_systems(
                OnEnter(SimState::Running),
                spawn_robot_camera.after(spawn_vehicle),
            );
    }
}
