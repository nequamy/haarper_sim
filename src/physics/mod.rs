use bevy::prelude::*;

pub mod battery;
pub mod collision;
pub mod dynamics;
pub mod motor;
pub mod pi_controller;
pub mod state;
pub mod tire_model;

use collision::sync_vehicle_after_collision;
use dynamics::{sync_vehicle_transform, update_physics};
use state::{VehicleParams, VehicleState};
use tire_model::TireParams;

use crate::physics::{
    battery::{BatteryParams, BatteryState},
    motor::{MotorParams, MotorState},
    pi_controller::PiController,
};

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(VehicleState::default())
            .insert_resource(VehicleParams::default())
            .insert_resource(TireParams::default())
            .insert_resource(MotorParams::default())
            .insert_resource(MotorState::default())
            .insert_resource(BatteryParams::default())
            .insert_resource(BatteryState::new())
            .insert_resource(Time::<Fixed>::from_hz(500.0))
            .insert_resource(PiController {
                kp: 0.23,
                ki: 0.075,
                saturation: 1.0,
                ..default()
            })
            .add_systems(
                FixedUpdate,
                (update_physics, sync_vehicle_transform).chain(),
            )
            .add_systems(PostUpdate, sync_vehicle_after_collision);
    }
}
