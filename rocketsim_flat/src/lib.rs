mod planus_flat;

pub use planus;
pub use planus_flat::rocketsim::*;

impl Color {
    pub const BLACK: Self = Self::rgb(0., 0., 0.);
    pub const WHITE: Self = Self::rgb(1., 1., 1.);
    pub const RED: Self = Self::rgb(1., 0., 0.);
    pub const GREEN: Self = Self::rgb(0., 1., 0.);
    pub const BLUE: Self = Self::rgb(0., 0., 1.);

    #[inline]
    #[must_use]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1. }
    }

    #[inline]
    #[must_use]
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }
}

impl From<Vec3> for glam::Vec3A {
    fn from(value: Vec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<glam::Vec3A> for Vec3 {
    fn from(value: glam::Vec3A) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<Mat3> for glam::Mat3A {
    fn from(value: Mat3) -> Self {
        Self {
            x_axis: value.forward.into(),
            y_axis: value.right.into(),
            z_axis: value.up.into(),
        }
    }
}

impl From<glam::Mat3A> for Mat3 {
    fn from(value: glam::Mat3A) -> Self {
        Self {
            forward: value.x_axis.into(),
            right: value.y_axis.into(),
            up: value.z_axis.into(),
        }
    }
}

impl From<WheelsWithContact> for [bool; 4] {
    fn from(value: WheelsWithContact) -> Self {
        [
            value.front_left,
            value.front_right,
            value.rear_left,
            value.rear_right,
        ]
    }
}

impl From<[bool; 4]> for WheelsWithContact {
    fn from(value: [bool; 4]) -> Self {
        Self {
            front_left: value[0],
            front_right: value[1],
            rear_left: value[2],
            rear_right: value[3],
        }
    }
}
