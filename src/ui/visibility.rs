use bevy::prelude::*;

#[derive(Resource)]
pub struct DebugVisibility {
    show_velocity: bool,
    show_tire_forces: bool,
    show_trail: bool,
    show_slip: bool,
    show_wireframe: bool,
    show_weight_transfer: bool,
}
