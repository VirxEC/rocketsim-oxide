use glam::{Affine3A, Vec3A};
use std::f32::consts::FRAC_1_SQRT_2;

pub(crate) mod aabb_util_2;
pub(crate) mod motion_state;
pub(crate) mod transform_util;

pub trait AffineTranspose {
    fn transpose(&self) -> Self;
    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A;
}

impl AffineTranspose for Affine3A {
    fn transpose(&self) -> Self {
        let matrix3 = self.matrix3.transpose();

        Self {
            matrix3,
            translation: matrix3 * -self.translation,
        }
    }

    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A {
        self.matrix3.transpose() * (in_vec - self.translation)
    }
}

pub fn plane_space(n: Vec3A) -> (Vec3A, Vec3A) {
    if n.z.abs() > FRAC_1_SQRT_2 {
        // choose p in y-z plane
        let a = n.y * n.y + n.z * n.z;
        let k = 1. / a.sqrt();
        let p = Vec3A::new(0., -n.z * k, n.y * k);
        (p, Vec3A::new(a * k, -n.x * p.z, n.x * p.y))
    } else {
        // choose p in x-y plane
        let a = n.x * n.x + n.y * n.y;
        let k = 1. / a.sqrt();
        let p = Vec3A::new(-n.y * k, n.x * k, 0.);
        (p, Vec3A::new(-n.z * p.y, n.z * p.x, a * k))
    }
}
