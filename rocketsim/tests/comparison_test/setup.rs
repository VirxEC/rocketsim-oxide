use glam::{Mat3A, Vec3A};
use rocketsim_rs::consts;
use serde::Serialize;
use rocketsim::{BallState, CarControls, CarState, Team};

#[derive(Debug, Clone, Copy, Serialize)]
pub struct CarSetup {
    pub team: Team,
    pub controls: CarControls,

    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
    pub on_ground: bool,

    pub boost: f32,
}

impl CarSetup {
    pub const fn new(team: Team, pos: Vec3A) -> Self {
        CarSetup {
            team,
            controls: CarControls::DEFAULT,
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            boost: consts::BOOST_MAX,
            on_ground: false
        }
    }

    pub const fn with_controls(mut self, controls: CarControls) -> Self {
        self.controls = controls;
        self
    }

    pub const fn with_rot(mut self, rot: Mat3A) -> Self {
        self.rot_mat = rot;
        self
    }

    pub const fn with_vel(mut self, vel: Vec3A) -> Self {
        self.vel = vel;
        self
    }

    pub const fn with_ang_vel(mut self, ang_vel: Vec3A) -> Self {
        self.ang_vel = ang_vel;
        self
    }

    pub const fn with_boost(mut self, boost: f32) -> Self {
        self.boost = boost;
        self
    }

    pub const fn with_on_ground(mut self, on_ground: bool) -> Self {
        self.on_ground = on_ground;
        self
    }

    pub fn make_car_state(&self) -> CarState {
        let mut result = CarState::DEFAULT;
        result.phys.pos = self.pos;
        result.phys.rot_mat = self.rot_mat;
        result.phys.vel = self.vel;
        result.phys.ang_vel = self.ang_vel;

        result.boost = self.boost;
        result
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct BallSetup {
    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl BallSetup {
    pub const fn new(pos: Vec3A) -> Self {
        BallSetup {
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        }
    }

    pub const fn with_rot(mut self, rot: Mat3A) -> Self {
        self.rot_mat = rot;
        self
    }

    pub const fn with_vel(mut self, vel: Vec3A) -> Self {
        self.vel = vel;
        self
    }

    pub const fn with_ang_vel(mut self, ang_vel: Vec3A) -> Self {
        self.ang_vel = ang_vel;
        self
    }

    pub fn make_ball_state(&self) -> BallState {
        let mut result = BallState::DEFAULT;
        result.phys.pos = self.pos;
        result.phys.rot_mat = self.rot_mat;
        result.phys.vel = self.vel;
        result.phys.ang_vel = self.ang_vel;
        result
    }
}