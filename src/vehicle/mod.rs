pub mod input;
pub mod reset;
pub mod spawn;
pub mod wheel;

use input::{VehicleInput, read_input};
use spawn::spawn_vehicle;
use wheel::{
    WheelAngleSpeed, WheelDynamics, update_forward_left_wheel_angle,
    update_forward_right_wheel_angle, update_wheel_angle,
};

use bevy::prelude::*;

use crate::vehicle::reset::reset_world;

pub struct VehiclePlugin;

impl Plugin for VehiclePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_vehicle)
            .insert_resource(VehicleInput::default())
            .insert_resource(WheelAngleSpeed::default())
            .insert_resource(WheelDynamics::new())
            .add_systems(Update, read_input)
            .add_systems(Update, reset_world.after(read_input))
            .add_systems(Update, update_wheel_angle)
            .add_systems(Update, update_forward_right_wheel_angle)
            .add_systems(Update, update_forward_left_wheel_angle);
    }
}
