use crate::{BoostPadConfig, BoostPadState, sim::consts};

#[derive(Debug, Copy, Clone)]
pub struct BoostPad {
    config: BoostPadConfig,
    radius: f32,
    pub(crate) internal_state: BoostPadState,
}

impl BoostPad {
    #[must_use]
    pub const fn new(config: BoostPadConfig) -> Self {
        let radius = if config.is_big {
            consts::boost_pads::BOX_RAD_BIG
        } else {
            consts::boost_pads::BOX_RAD_SMALL
        };

        Self {
            config,
            radius,
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

    pub const fn reset(&mut self) {
        self.internal_state = BoostPadState::DEFAULT;
    }

    #[must_use]
    pub const fn get_config(&self) -> &BoostPadConfig {
        &self.config
    }

    #[must_use]
    pub const fn get_radius(&self) -> f32 {
        self.radius
    }
}
