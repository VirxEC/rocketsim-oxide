use glam::Vec3A;

pub trait TriangleCallback {
    fn process_triangle(
        &mut self,
        triangle: &[Vec3A],
        part_id: usize,
        triangle_index: usize,
    ) -> bool;
}

pub trait InternalTriangleIndexCallback {
    fn internal_process_triangle_index(
        &mut self,
        triangle: &[Vec3A],
        tri_aabb_min: Vec3A,
        tri_aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool;
}
