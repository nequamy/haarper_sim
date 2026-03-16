use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::ui::visibility::DebugVisibility;

pub fn debug_panel(mut contexts: EguiContexts, mut vis: ResMut<DebugVisibility>) -> Result {
    let ctx = contexts.ctx_mut()?;

    egui::Window::new("Visibility")
        .default_pos([10.0, 10.0])
        .default_width(200.0)
        .resizable(false)
        .collapsible(true)
        .show(ctx, |ui| {
            ui.checkbox(&mut vis.show_velocity, "Velocity vector");
            ui.checkbox(&mut vis.show_slip, "Slip angle");
            ui.checkbox(&mut vis.show_trail, "Trail");
            ui.checkbox(&mut vis.show_tire_forces, "Tire forces");
            ui.checkbox(&mut vis.show_weight_transfer, "Weight transfer");
            ui.checkbox(&mut vis.show_wireframe, "Wireframe");
        });

    Ok(())
}
