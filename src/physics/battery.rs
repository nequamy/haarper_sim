use bevy::{math::FloatPow, prelude::*};

#[derive(Resource)]
pub struct BatteryParams {
    pub capacity_ah: f32,
    pub v_cell_full: f32,
    pub v_cell_nom: f32,
    pub v_cell_empty: f32,
    pub n_cells: u32,
    pub r: f32,
    pub k_polar: f32,
    pub k_r: f32,
    pub a: f32,
    pub b: f32,
}

impl Default for BatteryParams {
    fn default() -> Self {
        Self {
            capacity_ah: 2.3,
            v_cell_full: 4.2,
            v_cell_nom: 3.607,
            v_cell_empty: 3.2,
            n_cells: 3,
            r: 0.02,
            k_polar: 0.022,
            k_r: 0.02,
            a: 0.615,
            b: 3.0,
        }
    }
}

#[derive(Resource, Default)]
pub struct BatteryState {
    pub soc: f32,
    pub v_terminal: f32,
    pub i_draw: f32,
    pub r_internal: f32,
}

impl BatteryState {
    fn new() -> Self {
        Self {
            soc: 1.0,
            ..default()
        }
    }

    fn update(&mut self, current: f32, dt: f32, params: &BatteryParams) {
        self.i_draw = current;

        // Полином Шеферда
        let v_oc = params.v_cell_nom - params.k_polar / self.soc.max(0.001)
            + params.a * (-params.b * (1.0 - self.soc.max(0.001))).exp();
        let v_oc = v_oc.max(params.v_cell_empty);

        self.r_internal = params.r * (1.0 + params.k_r * (1.0 - self.soc).squared());

        self.v_terminal = params.n_cells as f32 * v_oc - self.i_draw * self.r_internal;

        self.soc -= current / (params.capacity_ah * 3600.0) * dt;
        self.soc = self.soc.clamp(0.05, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_update_battery_output_zero_current() {
        let mut battery = BatteryState::new();
        let params = BatteryParams::default();

        battery.update(0.0, 0.7, &params);

        assert_eq!(battery.soc, 1.0);
        assert!(12.6 - battery.v_terminal < 0.1);
        assert_eq!(battery.i_draw, 0.0);
    }

    #[test]
    fn test_update_battery_output_ten_current() {
        let mut battery = BatteryState::new();
        let params = BatteryParams::default();

        battery.update(10.0, 1.0, &params);

        assert!(battery.soc < 1.0);
        assert!(12.6 > battery.v_terminal);
        assert!(battery.i_draw == 10.0);
    }

    #[ignore = "Runtime test"]
    #[test]
    fn test_update_battery_output_low() {
        let mut battery = BatteryState::new();
        let params = BatteryParams::default();

        while battery.soc != 0.05 {
            battery.update(10.0, 1.0, &params);
        }
    }
}
