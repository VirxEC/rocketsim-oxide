use std::ops::Add;

use crate::bullet::linear_math::vector3::Vector3;

/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tri(pub [Vector3; 3]);

impl Tri {
    #[must_use]
    #[inline]
    /// Create a new triangle from 3 points
    pub const fn from_points(p0: Vector3, p1: Vector3, p2: Vector3) -> Self {
        Self([p0, p1, p2])
    }
}

// AABB stands for "Axis-Aligned Bounding Boxes"
// Learn more here: https://developer.nvidia.com/blog/thinking-parallel-part-i-collision-detection-gpu/
/// An axis-aligned bounding box.
#[derive(Clone, Copy, Debug, Default)]
pub struct Aabb {
    min: Vector3,
    max: Vector3,
}

impl Aabb {
    #[must_use]
    #[inline]
    #[allow(dead_code)]
    /// Create a new AABB.
    ///
    /// Used in tests.
    pub const fn new(min: Vector3, max: Vector3) -> Self {
        Self { min, max }
    }

    #[must_use]
    #[inline]
    /// The minimum point contained in the AABB.
    pub const fn min(self) -> Vector3 {
        self.min
    }

    #[must_use]
    #[inline]
    /// The maximum point contained in the AABB.
    pub const fn max(self) -> Vector3 {
        self.max
    }

    #[must_use]
    #[inline]
    /// Create an AABB from a triangle.
    pub fn from_tri(t: Tri) -> Self {
        Self {
            min: t.0.into_iter().reduce(Vector3::min).unwrap(),
            max: t.0.into_iter().reduce(Vector3::max).unwrap(),
        }
    }
}

impl Add for Aabb {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

impl From<Tri> for Aabb {
    #[inline]
    fn from(value: Tri) -> Self {
        Self::from_tri(value)
    }
}
