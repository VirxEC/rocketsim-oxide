use glam::{Affine3A, Mat3A, Quat, Vec3A, Vec4, Vec4Swizzles};
use std::f32::consts::FRAC_1_SQRT_2;

pub mod aabb_util_2;
pub mod transform_util;

pub const LARGE_FLOAT: f32 = 1e18;

pub trait AffineExt {
    fn transpose(&self) -> Self;
    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A;
}

impl AffineExt for Affine3A {
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

pub trait Mat3AExt {
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f32;
    fn bullet_inverse(&self) -> Self;
}

impl Mat3AExt for Mat3A {
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f32 {
        self.col(r1)[c1] * self.col(r2)[c2] - self.col(r1)[c2] * self.col(r2)[c1]
    }

    fn bullet_inverse(&self) -> Self {
        let co = Vec3A::new(
            self.cofac(1, 1, 2, 2),
            self.cofac(1, 2, 2, 0),
            self.cofac(1, 0, 2, 1),
        );
        let det = self.x_axis.dot(co);
        debug_assert_ne!(det, 0.0);
        let s = Vec3A::splat(det.recip());

        Self::from_cols(
            co * s,
            Vec3A::new(
                self.cofac(0, 2, 2, 1),
                self.cofac(0, 0, 2, 2),
                self.cofac(0, 1, 2, 0),
            ) * s,
            Vec3A::new(
                self.cofac(0, 1, 1, 2),
                self.cofac(0, 2, 1, 0),
                self.cofac(0, 0, 1, 1),
            ) * s,
        )
    }
}

pub trait QuatExt {
    fn bullet_mul_quat(self, q2: Self) -> Self;
}

impl QuatExt for Quat {
    fn bullet_mul_quat(self, q2: Self) -> Self {
        const NEG_W: Vec4 = Vec4::new(1.0, 1.0, 1.0, -1.0);

        let q1: Vec4 = self.into();
        let q2: Vec4 = q2.into();

        let a2 = q1.yzxy() * q2.zxyy();
        let a1 = q1.xyzx() * q2.wwwx() + a2;

        let b1 = q1.zxyz() * q2.yzxz();
        let a0 = q1.wwww() * q2 - b1;

        let q = a0 + a1 * NEG_W;
        Self::from_array(q.to_array())
    }
}

pub fn interpolate_3(v0: Vec3A, v1: Vec3A, rt: f32) -> Vec3A {
    let s = 1.0 - rt;
    s * v0 + rt * v1
}

pub fn plane_space(n: Vec3A) -> (Vec3A, Vec3A) {
    if n.z.abs() > FRAC_1_SQRT_2 {
        // choose p in y-z plane
        let a = n.y.mul_add(n.y, n.z * n.z);
        let k = 1. / a.sqrt();
        let p = Vec3A::new(0., -n.z * k, n.y * k);
        (p, Vec3A::new(a * k, -n.x * p.z, n.x * p.y))
    } else {
        // choose p in x-y plane
        let a = n.x.mul_add(n.x, n.y * n.y);
        let k = 1. / a.sqrt();
        let p = Vec3A::new(-n.y * k, n.x * k, 0.);
        (p, Vec3A::new(-n.z * p.y, n.z * p.x, a * k))
    }
}
