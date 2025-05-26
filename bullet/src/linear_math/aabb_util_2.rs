use glam::{U16Vec3, Vec3A};

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
