use glam::{Affine3A, U16Vec3, Vec3A};

#[inline]
pub fn test_aabb_against_aabb(
    aabb_min_1: Vec3A,
    aabb_max_1: Vec3A,
    aabb_min_2: Vec3A,
    aabb_max_2: Vec3A,
) -> bool {
    aabb_min_1.cmple(aabb_max_2).all() && aabb_max_1.cmpge(aabb_min_2).all()
}

#[inline]
pub fn test_quantized_aabb_against_quantized_aabb(
    aabb_min_1: U16Vec3,
    aabb_max_1: U16Vec3,
    aabb_min_2: U16Vec3,
    aabb_max_2: U16Vec3,
) -> bool {
    aabb_min_1.cmple(aabb_max_2).all() && aabb_max_1.cmpge(aabb_min_2).all()
}

pub fn transform_aabb(half_extents: Vec3A, margin: f32, t: &Affine3A) -> (Vec3A, Vec3A) {
    let half_extents_with_margin = half_extents + Vec3A::splat(margin);
    let abs_b = t.matrix3.abs();
    let center = t.translation;
    let extent = abs_b * half_extents_with_margin;

    (center - extent, center + extent)
}
