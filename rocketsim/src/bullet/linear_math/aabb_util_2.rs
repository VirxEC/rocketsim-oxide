use std::ops::{Add, AddAssign};

use glam::{Affine3A, Vec3A};

#[derive(Clone, Copy, Debug, Default)]
pub struct Aabb {
    pub min: Vec3A,
    pub max: Vec3A,
}

impl Aabb {
    pub const ZERO: Self = Self {
        min: Vec3A::ZERO,
        max: Vec3A::ZERO,
    };

    #[inline]
    pub const fn new(min: Vec3A, max: Vec3A) -> Self {
        Self { min, max }
    }

    #[inline]
    pub fn center(&self) -> Vec3A {
        (self.min + self.max) * 0.5
    }

    pub fn area(&self) -> f32 {
        let extents = self.max - self.min;
        2.0 * (extents.x * extents.y + extents.x * extents.z + extents.y * extents.z)
    }
}

impl Add for Aabb {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

impl AddAssign for Aabb {
    fn add_assign(&mut self, rhs: Self) {
        self.min = self.min.min(rhs.min);
        self.max = self.max.max(rhs.max);
    }
}

#[inline]
pub fn test_aabb_against_aabb(aabb_1: &Aabb, aabb_2: &Aabb) -> bool {
    aabb_1.min.cmple(aabb_2.max).all() && aabb_1.max.cmpge(aabb_2.min).all()
}

pub fn transform_aabb(half_extents: Vec3A, margin: f32, t: &Affine3A) -> Aabb {
    let half_extents_with_margin = half_extents + margin;
    let abs_b = t.matrix3.abs();
    let center = t.translation;
    let extent = abs_b * half_extents_with_margin;

    Aabb {
        min: center - extent,
        max: center + extent,
    }
}

pub fn intersect_ray_aabb(
    ray_from: Vec3A,
    ray_dir_inv: Vec3A,
    bounds: &Aabb,
    lambda_max: f32,
) -> bool {
    let t0 = (bounds.min - ray_from) * ray_dir_inv;
    let t1 = (bounds.max - ray_from) * ray_dir_inv;

    let t_enter = t0.min(t1);
    let t_exit = t0.max(t1);

    let t_near = t_enter.max_element();
    let t_far = t_exit.min_element();

    (t_near <= t_far) && (t_far >= 0.0) && (t_near <= lambda_max)
}
