use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::SimState;

const HEADER: &str = "
██╗  ██╗ █████╗  █████╗ ██████╗ ██████╗ ███████╗██████╗     ███████╗██╗███╗   ███╗
██║  ██║██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔════╝██╔══██╗    ██╔════╝██║████╗ ████║
███████║███████║███████║██████╔╝██████╔╝█████╗  ██████╔╝    ███████╗██║██╔████╔██║
██╔══██║██╔══██║██╔══██║██╔══██╗██╔═══╝ ██╔══╝  ██╔══██╗    ╚════██║██║██║╚██╔╝██║
██║  ██║██║  ██║██║  ██║██║  ██║██║     ███████╗██║  ██║    ███████║██║██║ ╚═╝ ██║
╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝    ╚══════╝╚═╝╚═╝     ╚═╝                                                                                  
";

#[derive(Resource, Default)]
pub struct SelectedTrack {
    pub path: String,
    pub name: String,
}

pub fn show_menu(
    mut contexts: EguiContexts,
    mut state: ResMut<NextState<SimState>>,
    mut track: ResMut<SelectedTrack>,
) -> Result {
    let ctx = contexts.ctx_mut()?;

    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(100.0);
            ui.monospace(
                egui::RichText::new(HEADER)
                    .monospace()
                    .color(egui::Color32::DARK_RED),
            );
            ui.add_space(30.0);

            ui.monospace("Select track:");
            if ui
                .selectable_label(
                    track.name == "Sochi",
                    egui::RichText::new("Sochi").monospace(),
                )
                .clicked()
            {
                track.name = "Sochi".into();
                track.path = "assets/tracks/sochi/Sochi_centerline.csv".into();
            }
            if ui
                .selectable_label(
                    track.name == "Spielberg",
                    egui::RichText::new("Spielberg").monospace(),
                )
                .clicked()
            {
                track.name = "Spielberg".into();
                track.path = "assets/tracks/spielberg/Spielberg_centerline.csv".into();
            }
            if ui
                .selectable_label(
                    track.name == "Void",
                    egui::RichText::new("Void").monospace(),
                )
                .clicked()
            {
                track.name = "Void".into();
                track.path = "void".into();
            }

            ui.add_space(20.0);
            let start_enabled = !track.path.is_empty();
            if ui
                .add_enabled(
                    start_enabled,
                    egui::Button::new(egui::RichText::new("Start").monospace()),
                )
                .clicked()
            {
                state.set(SimState::Running);
            }
        });
    });

    Ok(())
}

pub fn handle_file_drop(
    mut events: MessageReader<FileDragAndDrop>,
    mut track: ResMut<SelectedTrack>,
) {
    for event in events.read() {
        if let FileDragAndDrop::DroppedFile {
            window: _,
            path_buf,
        } = event
            && path_buf.extension().is_some_and(|ext| ext == "csv")
        {
            track.name = path_buf
                .file_stem()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            track.path = path_buf.to_string_lossy().into_owned();
        }
    }
}
