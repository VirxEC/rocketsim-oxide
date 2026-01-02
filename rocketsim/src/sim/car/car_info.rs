use crate::{CarBodyConfig, Team};

/// Immutable information attached to each car
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct CarInfo {
    pub idx: usize,
    pub team: Team,
    pub config: CarBodyConfig,
}