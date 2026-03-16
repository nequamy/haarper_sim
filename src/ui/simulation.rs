use bevy::prelude::*;

use crate::{
    physics::{battery::BatteryState, motor::MotorState, state::VehicleState},
    ui::DebugTextSimulation,
    vehicle::input::VehicleInput,
};

pub fn spawn_debug_ui_sim(mut commands: Commands) {
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

pub fn update_debug_ui_sim(
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
