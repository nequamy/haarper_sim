use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

pub mod forces;
pub mod gizmos;
pub mod panel;
pub mod simulation;
pub mod system;
pub mod visibility;

use crate::ui::{
    panel::debug_panel,
    simulation::{spawn_debug_ui_sim, update_debug_ui_sim},
    system::{
        ProcessDiagnostics, spawn_debug_ui_sys, update_debug_ui_sys, update_process_diagnostic,
    },
    visibility::DebugVisibility,
};

#[derive(Component)]
pub struct DebugTextSystem;

#[derive(Component)]
pub struct DebugTextSimulation;

pub struct DebugUIPlugin;

impl Plugin for DebugUIPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_plugins(EguiPlugin::default())
            .init_resource::<ProcessDiagnostics>()
            .insert_resource(DebugVisibility::default())
            .add_systems(Startup, spawn_debug_ui_sys)
            .add_systems(Startup, spawn_debug_ui_sim)
            .add_systems(EguiPrimaryContextPass, debug_panel)
            .add_systems(Update, update_debug_ui_sim)
            .add_systems(Update, update_debug_ui_sys)
            .add_systems(Update, update_process_diagnostic);
    }
}
