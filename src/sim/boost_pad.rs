use glam::Vec3A;

use crate::{UU_TO_BT, consts};

#[derive(Clone, Copy, Debug, Default)]
pub struct BoostPadConfig {
    pub pos: Vec3A,
    pub is_big: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BoostPadState {
    pub is_active: bool,
    pub cooldown: f32,
    pub cur_locked_car: u64,
    pub prev_locked_car_id: u64,
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    pub const DEFAULT: Self = Self {
        is_active: true,
        cooldown: 0.0,
        cur_locked_car: 0,
        prev_locked_car_id: 0,
    };
}

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
        let pos_bt = config.pos * UU_TO_BT;

        let box_rad = if config.is_big {
            consts::boostpads::BOX_RAD_BIG
        } else {
            consts::boostpads::BOX_RAD_SMALL
        } * UU_TO_BT;

        Self {
            config,
            pos_bt,
            box_min_bt: pos_bt - Vec3A::new(box_rad, box_rad, 0.0),
            box_max_bt: pos_bt
                + Vec3A::new(box_rad, box_rad, consts::boostpads::BOX_HEIGHT * UU_TO_BT),
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
