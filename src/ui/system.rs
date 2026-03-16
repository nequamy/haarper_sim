use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};

use std::process;
use sysinfo::{Pid, ProcessesToUpdate, System as SysInfoSystem};

use crate::ui::DebugTextSystem;

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

pub fn spawn_debug_ui_sys(mut commands: Commands) {
    commands
        .spawn(Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            padding: UiRect::all(Val::Px(8.0)),
            ..default()
        })
        .insert(BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)))
        .with_child((
            Text::new(""),
            TextFont {
                font_size: 14.0,
                ..default()
            },
            TextColor(Color::srgba(0.9, 0.9, 0.9, 1.0)),
            DebugTextSystem,
        ));
}

pub fn update_debug_ui_sys(
    time: Res<Time>,
    diagnostic: Res<DiagnosticsStore>,
    system: Res<ProcessDiagnostics>,
    mut query: Query<&mut Text, With<DebugTextSystem>>,
) {
    let Ok(mut text) = query.single_mut() else {
        return;
    };

    let fps = diagnostic
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    *text = Text::new(format!(
        "SYSTEM INFORMATION:\n\
        FPS: {fps:.0}\n\
            dt: {:.1} ms\n\
            CPU: {:.1}%\n\
            RAM: {:.0} MB",
        time.delta_secs() * 1000.0,
        system.cpu_usage,
        system.memory_mb,
    ))
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
