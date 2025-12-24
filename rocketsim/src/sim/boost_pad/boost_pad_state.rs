#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct BoostPadState {
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
        cooldown: 0.0,
        cur_locked_car: 0,
        prev_locked_car_id: 0,
    };

    pub fn is_active(&self) -> bool {
        self.cooldown == 0.0
    }
}