use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

pub mod gizmos;
pub mod menu;
pub mod panel;
pub mod resources;

use crate::{
    SimState,
    ui::{
        gizmos::{
            record_trail, update_forces_gizmos, update_slip_angle, update_slip_ratio,
            update_trail_gizmos, update_velocity_gizmos,
        },
        menu::{SelectedTrack, handle_file_drop, show_menu},
        panel::{debug_panel, update_process_diagnostic},
        resources::{DebugForces, DebugVisibility, ProcessDiagnostics, TrailHistory},
    },
};

pub struct DebugUIPlugin;

impl Plugin for DebugUIPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_plugins(EguiPlugin::default())
            .init_resource::<ProcessDiagnostics>()
            .insert_resource(SelectedTrack::default())
            .insert_resource(DebugVisibility::default())
            .insert_resource(DebugForces::default())
            .insert_resource(TrailHistory::default())
            .add_systems(
                EguiPrimaryContextPass,
                show_menu.run_if(in_state(SimState::Menu)),
            )
            .add_systems(
                EguiPrimaryContextPass,
                debug_panel.run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                update_velocity_gizmos.run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                update_forces_gizmos.run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                update_slip_angle.run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                update_slip_ratio.run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                (record_trail, update_trail_gizmos)
                    .chain()
                    .run_if(in_state(SimState::Running)),
            )
            .add_systems(
                Update,
                update_process_diagnostic.run_if(in_state(SimState::Running)),
            )
            .add_systems(Update, handle_file_drop.run_if(in_state(SimState::Menu)));
    }
}
