use std::ops::{Add, AddAssign};

use glam::{Affine3A, Vec3A, Vec4};

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

    #[inline]
    pub fn intersects(&self, rhs: &Self) -> bool {
        self.min.cmple(rhs.max).all() && self.max.cmpge(rhs.min).all()
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

/// Packet slab test for 4 rays using Vec4 lanes.
///
/// Returns a bitmask (bit i set if ray i overlaps the AABB).
/// Only the lowest 4 bits are set.
pub fn intersect_ray_aabb_packet(
    origins: &[Vec4; 3],
    inv_dir: &[Vec4; 3],
    bounds: &Aabb,
    lambda_max: Vec4,
) -> u8 {
    let t0x = (bounds.min.x - origins[0]) * inv_dir[0];
    let t1x = (bounds.max.x - origins[0]) * inv_dir[0];
    let tminx = t0x.min(t1x);
    let tmaxx = t0x.max(t1x);

    let t0y = (bounds.min.y - origins[1]) * inv_dir[1];
    let t1y = (bounds.max.y - origins[1]) * inv_dir[1];
    let tminy = t0y.min(t1y);
    let tmaxy = t0y.max(t1y);

    let t0z = (bounds.min.z - origins[2]) * inv_dir[2];
    let t1z = (bounds.max.z - origins[2]) * inv_dir[2];
    let tminz = t0z.min(t1z);
    let tmaxz = t0z.max(t1z);

    let t_enter = tminx.max(tminy).max(tminz);
    let t_exit = tmaxx.min(tmaxy).min(tmaxz);

    let cond1 = t_enter.cmple(t_exit);
    let cond2 = t_exit.cmpge(Vec4::ZERO);
    let cond3 = t_enter.cmple(lambda_max);

    (cond1 & cond2 & cond3).bitmask() as u8
}
