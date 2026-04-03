use bevy::prelude::*;
use std::{collections::VecDeque, process};
use sysinfo::{Pid, ProcessesToUpdate, System as SysInfoSystem};

#[derive(Resource, Default)]
pub struct DebugVisibility {
    pub show_velocity: bool,
    pub show_tire_forces: bool,
    pub show_trail: bool,
    pub show_slip: bool,
    pub show_lidar_ray: bool,
}

#[derive(Resource, Default)]
pub struct DebugForces {
    pub fx: [f32; 4],
    pub fy: [f32; 4],
    pub fz: [f32; 4],
    pub kappa: [f32; 4],
    pub alpha: [f32; 4],
    pub wheel_vx: [f32; 4],
    pub wheel_vy: [f32; 4],
}

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

#[derive(Resource, Default)]
pub struct TrailHistory {
    pub trail: VecDeque<Vec3>,
}
