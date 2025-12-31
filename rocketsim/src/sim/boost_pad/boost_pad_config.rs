use glam::Vec3A;
use crate::MutatorConfig;

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct BoostPadConfig {
    pub pos: Vec3A,
    pub is_big: bool,
}

impl BoostPadConfig {
    pub(crate) fn get_max_cooldown(&self, mutator_config: &MutatorConfig) -> f32 {
        if self.is_big {
            mutator_config.boost_pad_cooldown_big
        } else {
            mutator_config.boost_pad_cooldown_small
        }
    }

    pub(crate) fn get_boost_amount(&self, mutator_config: &MutatorConfig) -> f32 {
        if self.is_big {
            mutator_config.boost_pad_amount_big
        } else {
            mutator_config.boost_pad_amount_small
        }
    }
}
