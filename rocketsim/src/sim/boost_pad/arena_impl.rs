use crate::{Arena, BoostPadConfig, BoostPadState};

impl Arena {
    pub fn get_boost_pad_state(&self, idx: usize) -> BoostPadState {
        let pad = self.boost_pads()[idx];

        // TODO: Store a lot of these variables within the config maybe? Unsure
        let cooldown = if let Some(gave_boost_tick) = pad.gave_boost_tick_count {
            let max_cooldown = pad.config.get_max_cooldown(self.mutator_config());
            let time_since = (self.tick_count() - gave_boost_tick) as f32 * self.tick_time();
            (max_cooldown - time_since).max(0.0)
        } else {
            0.0
        };

        BoostPadState {
            cooldown,
        }
    }

    pub fn set_boost_pad_state(&mut self, idx: usize, state: BoostPadState) {
        todo!();
    }

    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }
}
