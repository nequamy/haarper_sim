use bevy::ecs::resource::Resource;

#[derive(Resource, Clone)]
pub struct TireParams {
    /// Коэффициент сцепления шины с дорогой
    mu: f32,
    /// Кривизна пика продольной силы
    ex: f32,
    /// Кривизная пика боковой силы
    ey: f32,
    /// Жесткость продольного скольжения
    bx: f32,
    /// Форма кривой продольной силы
    cx: f32,
    /// Жесткость бокового увода
    by: f32,
    /// Форма кривой боковой силы
    cy: f32,
    /// Чувствительность продольной силы к боковому скольжению
    bxa: f32,
    /// Форма ослабления продольной силы при боковом скольжении
    cxa: f32,
    /// Форма ослабления боковой силы к при продольном скольжении
    cyk: f32,
    /// Чувствительность боковой силы к продольному скольжению
    byk: f32,
}

impl Default for TireParams {
    fn default() -> Self {
        Self {
            mu: 0.9,
            ex: 0.0,
            ey: 0.0,
            bx: 10.0,
            cx: 1.65,
            by: 10.0,
            cy: 1.3,
            bxa: 12.0,
            cxa: 1.0,
            cyk: 1.0,
            byk: 10.0,
        }
    }
}

pub struct Pacejka {
    pub params: TireParams,
}

impl Pacejka {
    pub fn compute(&self, alpha: f32, kappa: f32, fz: f32) -> (f32, f32) {
        let fy0 = self.lateral(alpha, fz);
        let fx0 = self.longitudinal(kappa, fz);

        (fx0 * self.gxa(alpha), fy0 * self.gyk(kappa))
    }

    /// Функция для просчета чистой боковой силы одного колеса
    fn lateral(&self, alpha: f32, fz: f32) -> f32 {
        self.params.mu
            * fz
            * (self.params.cy * pacejka_base(self.params.by, self.params.ey, alpha)).sin()
    }

    /// Функция для расчета чистой продольной силы одного колеса
    fn longitudinal(&self, kappa: f32, fz: f32) -> f32 {
        self.params.mu
            * fz
            * (self.params.cx * pacejka_base(self.params.bx, self.params.ex, kappa)).sin()
    }

    /// Коэффициент ослабления продольной силы из-за бокового скольжения
    fn gxa(&self, alpha: f32) -> f32 {
        (self.params.cxa * (self.params.bxa * alpha).atan()).cos()
    }

    /// Коэффициент ослабления боковой силы из-за продольного скольжения
    fn gyk(&self, kappa: f32) -> f32 {
        (self.params.cyk * (self.params.byk * kappa).atan()).cos()
    }
}

fn pacejka_base(b: f32, e: f32, x: f32) -> f32 {
    (b * x - e * (b * x - (b * x).atan())).atan()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_correction() {
        let pac = Pacejka {
            params: TireParams::default(),
        };

        let (fx, fy) = pac.compute(0.05, 0.0, 9.8);

        assert!(fx == 0.0 && fy != 0.0);
    }

    #[test]
    fn test_compute_pacejka() {
        let pac = Pacejka {
            params: TireParams::default(),
        };

        assert_eq!(pac.compute(0.0, 0.0, 0.0), (0.0, 0.0))
    }

    #[test]
    fn test_high_x_force() {
        let pac = Pacejka {
            params: TireParams::default(),
        };

        let (fx, fy) = pac.compute(100.0, 12.0, 0.12);

        assert!(fx.abs() <= pac.params.mu * 0.12);
        assert!(fy.abs() <= pac.params.mu * 0.12);
    }

    #[test]
    fn test_g_boundaries() {
        let pac = Pacejka {
            params: TireParams::default(),
        };

        for i in -1000..=1000 {
            let alpha = i as f32 * 0.001;
            assert!(pac.gxa(alpha) <= 1.0 && pac.gxa(alpha) >= 0.0);
            assert!(pac.gyk(alpha) <= 1.0 && pac.gyk(alpha) >= 0.0);
        }
    }

    #[test]
    fn test_lateral_force_sign() {
        let pac = Pacejka {
            params: TireParams::default(),
        };

        let alpha = 0.6;
        let fz = 9.8;

        assert_eq!(pac.lateral(alpha, fz).signum(), alpha.signum());
        assert_eq!(pac.lateral(-alpha, fz).signum(), -alpha.signum());
    }

    #[test]
    fn test_longitudinal_force_sign() {
        let pac = Pacejka {
            params: TireParams::default(),
        };

        let kappa = 0.6;
        let fz = 9.8;

        assert_eq!(pac.longitudinal(kappa, fz).signum(), kappa.signum());
        assert_eq!(pac.longitudinal(-kappa, fz).signum(), -kappa.signum());
    }
}
