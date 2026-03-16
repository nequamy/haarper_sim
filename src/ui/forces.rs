use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct DebugForces {
    pub fx: [f32; 4],
    pub fy: [f32; 4],
    pub fz: [f32; 4],
    pub kappa: [f32; 4],
    pub alpha: [f32; 4],
}
