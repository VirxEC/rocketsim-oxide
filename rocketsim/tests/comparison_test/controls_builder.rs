use rocketsim::CarControls;

pub struct ControlsBuilder {
    car_controls: CarControls,
}

impl ControlsBuilder {
    pub const fn new() -> Self {
        Self {
            car_controls: CarControls::DEFAULT,
        }
    }

    pub const fn with_throttle(mut self, val: f32) -> Self {
        self.car_controls.throttle = val;
        self
    }
    pub const fn with_steer(mut self, val: f32) -> Self {
        self.car_controls.steer = val;
        self
    }

    pub const fn with_pitch(mut self, val: f32) -> Self {
        self.car_controls.pitch = val;
        self
    }
    pub const fn with_yaw(mut self, val: f32) -> Self {
        self.car_controls.yaw = val;
        self
    }
    pub const fn with_roll(mut self, val: f32) -> Self {
        self.car_controls.roll = val;
        self
    }

    pub const fn with_jump(mut self, val: bool) -> Self {
        self.car_controls.jump = val;
        self
    }
    pub const fn with_boost(mut self, val: bool) -> Self {
        self.car_controls.boost = val;
        self
    }
    pub const fn with_handbrake(mut self, val: bool) -> Self {
        self.car_controls.handbrake = val;
        self
    }

    pub const fn build(&self) -> CarControls {
        self.car_controls
    }
}
