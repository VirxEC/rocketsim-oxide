#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct CarControls {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

impl Default for CarControls {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarControls {
    pub const DEFAULT: Self = Self {
        throttle: 0.0,
        steer: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
        jump: false,
        boost: false,
        handbrake: false,
    };

    pub fn clamp(&self) -> CarControls {
        let mut result = self.clone();
        result.throttle = result.throttle.clamp(-1.0, 1.0);
        result.steer = result.steer.clamp(-1.0, 1.0);
        result.pitch = result.pitch.clamp(-1.0, 1.0);
        result.yaw = result.yaw.clamp(-1.0, 1.0);
        result.roll = result.roll.clamp(-1.0, 1.0);
        result
    }
}
