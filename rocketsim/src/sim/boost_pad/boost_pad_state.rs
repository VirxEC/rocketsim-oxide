#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct BoostPadState {
    // TODO: Implement car-locking to improve accuracy under certain conditions
    pub cooldown: f32,
    pub gave_car_boost: bool,
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    pub const DEFAULT: Self = Self {
        cooldown: 0.0,
        gave_car_boost: false,
    };

    pub fn is_active(&self) -> bool {
        self.cooldown == 0.0
    }
}
