use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};
use std::process;
use sysinfo::{Pid, ProcessesToUpdate, System as SysInfoSystem};

use crate::physics::{battery::BatteryState, motor::MotorState, state::VehicleState};
use crate::vehicle::input::VehicleInput;

#[derive(Resource)]
struct ProcessDiagnostics {
    sys: SysInfoSystem,
    pid: Pid,
    cpu_usage: f32,
    memory_mb: f64,
    update_timer: Timer,
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

#[derive(Component)]
struct DebugTextSystem;

#[derive(Component)]
struct DebugTextSimulation;

pub struct DebugUIPlugin;

impl Plugin for DebugUIPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .init_resource::<ProcessDiagnostics>()
            .add_systems(Update, update_process_diagnostic)
            .add_systems(Startup, spawn_debug_ui)
            .add_systems(Update, update_debug_ui_sim)
            .add_systems(Update, update_debug_ui_sys);
    }
}

fn spawn_debug_ui(mut commands: Commands) {
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

    commands
        .spawn(Node {
            position_type: PositionType::Absolute,
            top: Val::Px(120.0),
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
            DebugTextSimulation,
        ));
}

fn update_debug_ui_sys(
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

fn update_debug_ui_sim(
    state: Res<VehicleState>,
    input: Res<VehicleInput>,
    battery: Res<BatteryState>,
    motor: Res<MotorState>,
    mut query: Query<&mut Text, With<DebugTextSimulation>>,
) {
    let Ok(mut text) = query.single_mut() else {
        return;
    };

    *text = Text::new(format!(
        "SIMULATION INFORMATION:\n\
            pos: ({:.2}, {:.2})\n\
            yaw: {:.1}\n\
            vx: {:.2} m/s\n\
            vy: {:.2} m/s\n\
            omega: {:.2} rad/s\n\
            steering: {:.1}\n\
            throttle: {:.2}\n\
            \n\
            rpm: {:.2}\n\
            current: {:.2}\n\
            voltage: {:.2}",
        state.x,
        state.y,
        state.yaw.to_degrees(),
        state.vx,
        state.vy,
        state.omega,
        input.steering.to_degrees(),
        input.throttle,
        motor.rpm,
        motor.current,
        battery.v_terminal
    ))
}

fn update_process_diagnostic(time: Res<Time>, mut diag: ResMut<ProcessDiagnostics>) {
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
