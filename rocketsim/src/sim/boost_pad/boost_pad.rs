use glam::Vec3A;

use crate::{BoostPadConfig, BoostPadState, sim::consts};

#[derive(Debug, Copy, Clone)]
pub struct BoostPad {
    config: BoostPadConfig,
    radius: f32,
    aabb: (Vec3A, Vec3A),
    internal_state: BoostPadState,
}

impl BoostPad {
    #[must_use]
    pub fn new(config: BoostPadConfig) -> Self {
        let radius = if config.is_big {
            consts::boost_pads::BOX_RAD_BIG
        } else {
            consts::boost_pads::BOX_RAD_SMALL
        };

        let aabb = (
            config.pos - Vec3A::new(radius, radius, 0.0),
            config.pos + Vec3A::new(radius, radius, consts::boost_pads::BOX_HEIGHT),
        );

        Self {
            config,
            radius,
            aabb,
            internal_state: BoostPadState::DEFAULT,
        }
    }

    #[must_use]
    pub const fn get_state(&self) -> &BoostPadState {
        &self.internal_state
    }

    pub const fn set_state(&mut self, state: BoostPadState) {
        self.internal_state = state;
    }

    #[must_use]
    pub const fn get_config(&self) -> &BoostPadConfig {
        &self.config
    }

    pub const fn get_radius(&self) -> f32 {
        self.radius
    }
}
