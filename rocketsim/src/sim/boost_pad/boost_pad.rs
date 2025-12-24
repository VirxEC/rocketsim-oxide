use glam::Vec3A;
use crate::{BoostPadConfig, BoostPadState};
use crate::sim::consts;

#[allow(unused)]
pub struct BoostPad {
    config: BoostPadConfig,
    pos_bt: Vec3A,
    box_min_bt: Vec3A,
    box_max_bt: Vec3A,
    internal_state: BoostPadState,
}

impl BoostPad {
    #[must_use]
    pub fn new(config: BoostPadConfig) -> Self {
        let pos_bt = config.pos * consts::UU_TO_BT;

        let box_rad = if config.is_big {
            consts::boost_pads::BOX_RAD_BIG
        } else {
            consts::boost_pads::BOX_RAD_SMALL
        } * consts::UU_TO_BT;

        Self {
            config,
            pos_bt,
            box_min_bt: pos_bt - Vec3A::new(box_rad, box_rad, 0.0),
            box_max_bt: pos_bt
                + Vec3A::new(
                    box_rad,
                    box_rad,
                    consts::boost_pads::BOX_HEIGHT * consts::UU_TO_BT,
                ),
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
}
