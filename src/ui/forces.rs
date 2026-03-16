use bevy::prelude::*;

pub struct DebugForces {
    fx: [f32; 4],
    fy: [f32; 4],
    kappa: [f32; 4],
    alpha: [f32; 4],
    positions: [Vec2; 4],
}
