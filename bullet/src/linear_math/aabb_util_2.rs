use glam::{U16Vec3, Vec3A};

pub fn test_triangle_against_aabb2(vertices: &[Vec3A], aabb_min: &Vec3A, aabb_max: &Vec3A) -> bool {
    // First check Z, then X, then Y
    const INDEX_ORDER: [usize; 3] = [2, 0, 1];

    let p1 = vertices[0];
    let p2 = vertices[1];
    let p3 = vertices[2];

    for &i in &INDEX_ORDER {
        if p1[i].min(p2[i]).min(p3[i]) > aabb_max[i] {
            return false;
        }

        if p1[i].max(p2[i]).max(p3[i]) < aabb_min[i] {
            return false;
        }
    }

    true
}

#[inline]
pub fn test_quantized_aabb_against_quantized_aabb(
    aabb_min_1: U16Vec3,
    aabb_max_1: U16Vec3,
    aabb_min_2: U16Vec3,
    aabb_max_2: U16Vec3,
) -> bool {
    !(aabb_min_1.cmpgt(aabb_max_2).any() || aabb_max_1.cmplt(aabb_min_2).any())
}
