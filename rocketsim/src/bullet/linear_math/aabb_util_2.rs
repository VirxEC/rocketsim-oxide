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

pub fn ray_aabb_2(
    ray_from: Vec3A,
    ray_inv_dir: Vec3A,
    ray_sign: &[usize; 3],
    bounds: &[Vec3A; 2],
    lambda_max: f32,
) -> bool {
    let mut tmin = (bounds[ray_sign[0]].x - ray_from.x) * ray_inv_dir.x;
    let mut tmax = (bounds[1 - ray_sign[0]].x - ray_from.x) * ray_inv_dir.x;
    let tymin = (bounds[ray_sign[1]].y - ray_from.y) * ray_inv_dir.y;
    let tymax = (bounds[1 - ray_sign[1]].y - ray_from.y) * ray_inv_dir.y;

    if tmin > tymax || tymin > tmax {
        return false;
    }

    tmin = tmin.max(tymin);
    tmax = tmax.min(tymax);

    let tzmin = (bounds[ray_sign[2]].z - ray_from.z) * ray_inv_dir.z;
    let tzmax = (bounds[1 - ray_sign[2]].z - ray_from.z) * ray_inv_dir.z;

    if tmin > tzmax || tzmin > tmax {
        return false;
    }

    tmin = tmin.max(tzmin);
    tmax = tmax.min(tzmax);

    tmin < lambda_max && tmax > 0.0
}
