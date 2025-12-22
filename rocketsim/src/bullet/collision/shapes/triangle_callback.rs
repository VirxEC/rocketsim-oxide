use super::triangle_shape::TriangleShape;
use crate::bullet::linear_math::aabb_util_2::Aabb;

pub trait TriangleCallback {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb: &Aabb,
        triangle_index: usize,
    ) -> bool;
}

pub trait TriangleRayPacketCallback {
    fn process_node(
        &mut self,
        triangle: &TriangleShape,
        active_mask: u8,
        lambda_max: &mut [f32; 4],
    );
}
