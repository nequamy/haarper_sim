use bevy::prelude::*;

use crate::{
    SimState,
    comms::channels::{
        CommsChannels, apply_external_commands, publish_imu, publish_lidar, publish_vesc,
    },
    vehicle::input::read_input,
};

pub mod channels;
pub mod encoders;
pub mod sockets;

pub struct CommsPlugin;

impl Plugin for CommsPlugin {
    fn build(&self, app: &mut App) {
        let channels = CommsChannels::new();

        sockets::imu::spawn_imu_thread(channels.imu_rx.clone(), channels.imu_connected.clone());
        sockets::lidar::spawn_lidar_thread(
            channels.lidar_rx.clone(),
            channels.lidar_connected.clone(),
        );
        sockets::vesc::spawn_vesc_thread(channels.vesc_rx.clone(), channels.vesc_connected.clone());
        sockets::cmd::spawn_cmd_thread(channels.cmd_tx.clone());

        app.insert_resource(channels)
            .add_systems(
                Update,
                (publish_imu, publish_lidar, publish_vesc).run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                apply_external_commands
                    .after(read_input)
                    .run_if(in_state(SimState::Running)),
            );
    }
}
