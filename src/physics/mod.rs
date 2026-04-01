use bevy::prelude::*;

pub mod battery;
pub mod collision;
pub mod dynamics;
pub mod motor;
pub mod servo;
pub mod state;
pub mod tire_model;

use collision::sync_vehicle_after_collision;
use dynamics::{sync_vehicle_transform, update_physics};
use state::{VehicleParams, VehicleState};
use tire_model::TireParams;

use crate::{
    SimState,
    physics::{
        battery::{BatteryParams, BatteryState},
        motor::{MotorParams, MotorState},
        servo::{ServoParams, ServoState},
    },
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
            .insert_resource(ServoParams::default())
            .insert_resource(ServoState::default())
            .insert_resource(Time::<Fixed>::from_hz(500.0))
            .add_systems(
                FixedUpdate,
                (update_physics, sync_vehicle_transform)
                    .chain()
                    .run_if(in_state(SimState::Running)),
            )
            .add_systems(
                PostUpdate,
                sync_vehicle_after_collision.run_if(in_state(SimState::Running)),
            );
    }
}
