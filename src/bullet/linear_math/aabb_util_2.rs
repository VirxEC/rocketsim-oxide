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

pub fn ray_aabb_2(
    ray_from: Vec3A,
    ray_inv_dir: Vec3A,
    ray_sign: [bool; 3],
    bounds: &[Vec3A; 2],
    lambda_min: f32,
    lambda_max: f32,
) -> bool {
    let mut tmin = (bounds[ray_sign[0] as usize].x - ray_from.x) * ray_inv_dir.x;
    let mut tmax = (bounds[!ray_sign[0] as usize].x - ray_from.x) * ray_inv_dir.x;
    let tymin = (bounds[ray_sign[1] as usize].y - ray_from.y) * ray_inv_dir.y;
    let tymax = (bounds[!ray_sign[1] as usize].y - ray_from.y) * ray_inv_dir.y;

    if tmin > tymax || tymin > tmax {
        return false;
    }

    if tymin > tmin {
        tmin = tymin;
    }

    if tymax < tmax {
        tmax = tymax;
    }

    let tzmin = (bounds[ray_sign[2] as usize].z - ray_from.z) * ray_inv_dir.z;
    let tzmax = (bounds[!ray_sign[2] as usize].z - ray_from.z) * ray_inv_dir.z;

    if tmin > tzmax || tzmin > tmax {
        return false;
    }

    if tzmin > tmin {
        tmin = tzmin;
    }

    if tzmax < tmax {
        tmax = tzmax;
    }

    tmin < lambda_max && tmax > lambda_min
}
