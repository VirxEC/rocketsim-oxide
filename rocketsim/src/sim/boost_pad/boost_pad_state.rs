#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct BoostPadState {
    /// The last tick when we gave a car boost
    pub gave_boost_tick_count: Option<u64>, // TODO: Implement car-locking to improve accuracy under certain conditions
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    pub const DEFAULT: Self = Self {
        gave_boost_tick_count: None,
    };
}
