use glam::{Mat3A, Vec3A};

/// Default is not implemented for this struct,
/// because the initial start height of the ball/car is different.
/// The correct values are set in `BallState::default()` and `CarState::default()`
#[derive(Clone, Copy, Debug)]
pub struct PhysState {
    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl PhysState {
    #[must_use]
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