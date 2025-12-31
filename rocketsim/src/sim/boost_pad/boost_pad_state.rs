#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct BoostPadState {
    /// The last tick when we gave a car boost
    pub cooldown: f32,
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    pub const DEFAULT: Self = Self {
        cooldown: 0.0,
    };

    pub const fn is_active(&self) -> bool {
        self.cooldown <= 0.0
    }
}
