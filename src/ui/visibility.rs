use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct DebugVisibility {
    pub show_velocity: bool,
    pub show_tire_forces: bool,
    pub show_trail: bool,
    pub show_slip: bool,
    pub show_wireframe: bool,
    pub show_weight_transfer: bool,
}
