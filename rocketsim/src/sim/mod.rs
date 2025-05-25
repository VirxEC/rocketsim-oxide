mod arena;
mod ball;
mod boost_pad;
pub(crate) mod boost_pad_grid;
mod mutator_config;

pub use arena::*;
pub use ball::*;
pub use boost_pad::*;
pub use mutator_config::*;

use glam::{Mat3A, Vec3A};

pub(crate) enum CollisionMasks {
    HoopsNet = (1 << 8),
    DropshotTile = (1 << 9),
    DropshotFloor = (1 << 10),
}

#[derive(Clone, Copy, Debug)]
pub struct PhysState {
    pos: Vec3A,
    rot_mat: Mat3A,
    vel: Vec3A,
    ang_vel: Vec3A,
}

impl Default for PhysState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl PhysState {
    pub const DEFAULT: Self = Self {
        pos: Vec3A::ZERO,
        rot_mat: Mat3A::IDENTITY,
        vel: Vec3A::ZERO,
        ang_vel: Vec3A::ZERO,
    };

    pub fn get_inverted_y(mut self) -> Self {
        const INVERT_SCALE: Vec3A = Vec3A::new(-1.0, -1.0, 1.0);

        self.pos *= INVERT_SCALE;
        self.vel *= INVERT_SCALE;
        self.ang_vel *= INVERT_SCALE;

        for i in 0..3 {
            *self.rot_mat.col_mut(i) *= INVERT_SCALE;
        }

        self
    }
}
