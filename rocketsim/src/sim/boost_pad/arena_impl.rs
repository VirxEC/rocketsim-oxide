use crate::{Arena, BoostPadConfig, BoostPadState};

impl Arena {
    pub fn get_boost_pad_state(&self, idx: usize) -> &BoostPadState {
        &self.boost_pads()[idx].internal_state
    }

    pub fn set_boost_pad_state(&mut self, idx: usize, state: BoostPadState) {
        self.boost_pads_mut()[idx].internal_state = state
    }

    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }
}