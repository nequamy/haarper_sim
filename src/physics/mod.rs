use bevy::prelude::*;

pub mod collision;
pub mod dynamics;
pub mod state;
pub mod tire_model;

use collision::sync_vehicle_after_collision;
use dynamics::{sync_vehicle_transform, update_physics};
use state::{VehicleParams, VehicleState};
use tire_model::TireParams;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.insert_resource(VehicleState::default())
            .insert_resource(VehicleParams::default())
            .insert_resource(TireParams::default())
            .add_systems(
                FixedUpdate,
                (update_physics, sync_vehicle_transform).chain(),
            )
            .add_systems(PostUpdate, sync_vehicle_after_collision);
    }
}
