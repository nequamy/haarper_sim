use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

pub mod forces;
pub mod gizmos;
pub mod panel;
pub mod visibility;

use crate::ui::{
    forces::DebugForces,
    gizmos::{update_forces_gizmos, update_velocity_gizmos},
    panel::{ProcessDiagnostics, debug_panel, update_process_diagnostic},
    visibility::DebugVisibility,
};

pub struct DebugUIPlugin;

impl Plugin for DebugUIPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_plugins(EguiPlugin::default())
            .init_resource::<ProcessDiagnostics>()
            .insert_resource(DebugVisibility::default())
            .insert_resource(DebugForces::default())
            .add_systems(EguiPrimaryContextPass, debug_panel)
            .add_systems(Update, update_velocity_gizmos)
            .add_systems(Update, update_forces_gizmos)
            .add_systems(Update, update_process_diagnostic);
    }
}
