use std::process;

use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};
use bevy_egui::{EguiContexts, egui};
use sysinfo::{Pid, ProcessesToUpdate, System as SysInfoSystem};

use crate::{
    physics::{battery::BatteryState, motor::MotorState, state::VehicleState},
    ui::visibility::DebugVisibility,
    vehicle::input::VehicleInput,
};

#[derive(Resource)]
pub struct ProcessDiagnostics {
    pub sys: SysInfoSystem,
    pub pid: Pid,
    pub cpu_usage: f32,
    pub memory_mb: f64,
    pub update_timer: Timer,
}

impl Default for ProcessDiagnostics {
    fn default() -> Self {
        let mut sys = SysInfoSystem::new();
        let pid = Pid::from_u32(process::id());
        sys.refresh_processes(ProcessesToUpdate::Some(&[pid]), false);
        Self {
            sys,
            pid,
            cpu_usage: 0.0,
            memory_mb: 0.0,
            update_timer: Timer::from_seconds(1.0, TimerMode::Repeating),
        }
    }
}

pub fn debug_panel(
    vehicle_state: Res<VehicleState>,
    input: Res<VehicleInput>,
    motor_state: Res<MotorState>,
    battery_state: Res<BatteryState>,
    process: Res<ProcessDiagnostics>,
    diag_store: Res<DiagnosticsStore>,
    time: Res<Time>,
    mut contexts: EguiContexts,
    mut vis: ResMut<DebugVisibility>,
) -> Result {
    let ctx = contexts.ctx_mut()?;

    ctx.style_mut(|style| {
        style.visuals.window_fill = egui::Color32::from_black_alpha(180);
        style.visuals.window_stroke = egui::Stroke::new(1.0, egui::Color32::from_gray(60))
    });

    egui::Window::new("System")
        .anchor(egui::Align2::LEFT_TOP, [5.0, 5.0])
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            egui::Grid::new("system_grid")
                .num_columns(4)
                .spacing([10.0, 2.0])
                .show(ui, |ui| {
                    ui.monospace("FPS:");
                    ui.monospace(format!(
                        "{:.0}",
                        diag_store
                            .get(&FrameTimeDiagnosticsPlugin::FPS)
                            .and_then(|d| d.smoothed())
                            .unwrap_or(0.0)
                    ));
                    ui.monospace("dt:");
                    ui.monospace(format!("{:.1} ms", time.delta_secs() * 1000.0));
                    ui.end_row();

                    ui.monospace("CPU:");
                    ui.monospace(format!("{:.1}%", process.cpu_usage));
                    ui.monospace("RAM:");
                    ui.monospace(format!("{:.0} MB", process.memory_mb));
                    ui.end_row();
                });
        });

    egui::Window::new("Simulation")
        .anchor(egui::Align2::LEFT_TOP, [5.0, 90.0])
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            egui::Grid::new("simulation_grid")
                .num_columns(4)
                .spacing([20.0, 4.0])
                .show(ui, |ui| {
                    ui.monospace("Position:");
                    ui.end_row();
                    ui.monospace(format!("x: {:.2}", vehicle_state.x));
                    ui.monospace(format!("y: {:.2}", vehicle_state.y));
                    ui.end_row();

                    ui.end_row();
                    ui.monospace("Velocity:");
                    ui.end_row();
                    ui.monospace(format!("vx: {:.2}", vehicle_state.vx));
                    ui.monospace(format!("vy: {:.2}", vehicle_state.vy));
                    ui.end_row();

                    ui.end_row();
                    ui.monospace("Angular:");
                    ui.end_row();
                    ui.monospace(format!("yaw: {:.1}", vehicle_state.yaw.to_degrees()));
                    ui.monospace(format!("omega: {:.2}", vehicle_state.omega));
                    ui.end_row();

                    ui.end_row();
                    ui.monospace("Control:");
                    ui.end_row();
                    ui.monospace(format!("throttle: {:.2}", input.throttle));
                    ui.monospace(format!("steering: {:.1}", input.steering));
                    ui.end_row();

                    ui.end_row();
                    ui.monospace("Analog:");
                    ui.end_row();
                    ui.monospace(format!("RPM: {:.0}", motor_state.rpm));
                    ui.end_row();
                    ui.monospace(format!("current: {:.2}", motor_state.current));
                    ui.end_row();
                    ui.monospace(format!("voltage: {:.2}", battery_state.v_terminal));
                    ui.end_row();
                });
        });

    egui::Window::new("Visibility")
        .anchor(egui::Align2::RIGHT_TOP, [-5.0, 5.0])
        .min_width(180.0)
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

pub fn update_process_diagnostic(time: Res<Time>, mut diag: ResMut<ProcessDiagnostics>) {
    diag.update_timer.tick(time.delta());
    if !diag.update_timer.just_finished() {
        return;
    }

    let pid = diag.pid;
    diag.sys
        .refresh_processes(ProcessesToUpdate::Some(&[pid]), false);

    let cpu = diag.sys.process(pid).map(|p| p.cpu_usage()).unwrap_or(0.0) as f32 / 12.0;
    let mem = diag.sys.process(pid).map(|p| p.memory()).unwrap_or(0) as f64 / (1024.0 * 1024.0);

    diag.cpu_usage = cpu;
    diag.memory_mb = mem;
}
