use glam::Vec3A;
use crate::{BoostPadConfig, BoostPadState, sim::consts, MutatorConfig};
use crate::consts::boost_pads;
use crate::shared::Aabb;

#[derive(Debug, Copy, Clone)]
pub(crate) struct BoostPad {
    pub config: BoostPadConfig,
    radius: f32,
    aabb: Aabb,
    pub gave_boost_tick_count: Option<u64>, // TODO: Implement car-locking to improve accuracy under certain conditions
}

impl BoostPad {
    #[must_use]
    pub fn new(config: BoostPadConfig) -> Self {
        let radius = if config.is_big {
            boost_pads::BOX_RAD_BIG
        } else {
            boost_pads::BOX_RAD_SMALL
        };

        let box_rad = if config.is_big { boost_pads::BOX_RAD_BIG } else { boost_pads::BOX_RAD_SMALL };
        let extent = Vec3A::new(box_rad, box_rad, boost_pads::CYL_HEIGHT);
        let aabb = Aabb::new(config.pos - extent, config.pos + extent);

        Self {
            config,
            radius,
            aabb,
            gave_boost_tick_count: None,
        }
    }

    pub const fn reset(&mut self) {
        self.gave_boost_tick_count = None;
    }

    #[must_use]
    pub const fn config(&self) -> &BoostPadConfig {
        &self.config
    }

    #[must_use]
    pub const fn radius(&self) -> f32 {
        self.radius
    }

    #[must_use]
    pub const fn aabb(&self) -> Aabb { self.aabb }
}
