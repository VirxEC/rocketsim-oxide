use glam::Vec3A;

use crate::{
    GameMode,
    sim::{BallState, BoostPadConfig, BoostPadState, CarConfig, CarState, Team},
};

#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub enum TileState {
    #[default]
    Full,
    Damaged,
    Broken,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DropshotTile {
    pub pos: Vec3A,
    pub state: TileState,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BoostPadInfo {
    pub config: BoostPadConfig,
    pub state: BoostPadState,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CarInfo {
    pub id: u64,
    pub team: Team,
    pub state: CarState,
    pub config: CarConfig,
}

#[derive(Clone, Debug, Default)]
pub struct GameState {
    pub tick_rate: f32,
    pub tick_count: u64,
    pub game_mode: GameMode,
    pub cars: Option<Vec<CarInfo>>,
    pub ball: BallState,
    pub pads: Option<Vec<BoostPadInfo>>,
    pub tiles: Option<[Vec<DropshotTile>; 2]>,
}
