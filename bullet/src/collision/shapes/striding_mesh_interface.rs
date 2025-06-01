use super::{triangle_callback::InternalTriangleIndexCallback, triangle_shape::TriangleShape};
use glam::Vec3A;

pub trait StridingMeshInterface {
    fn internal_process_all_triangles(
        &self,
        callback: &mut dyn InternalTriangleIndexCallback,
        _aabb_min: &Vec3A,
        _aabb_max: &Vec3A,
    ) {
        for part in 0..self.get_num_sub_parts() {
            let (tris, aabbs) = self.get_tris_aabbs(part);

            for (i, (triangle, (aabb_min, aabb_max))) in tris.iter().zip(aabbs).enumerate() {
                let continue_processing = callback
                    .internal_process_triangle_index(triangle, *aabb_min, *aabb_max, part, i);
                if !continue_processing {
                    return;
                }
            }
        }
    }

    fn get_triangle(&self, subpart: usize, index: usize) -> TriangleShape {
        self.get_tris_aabbs(subpart).0[index]
    }

    fn get_total_num_faces(&self) -> usize;

    fn get_tris_aabbs(&self, subpart: usize) -> (&[TriangleShape], &[(Vec3A, Vec3A)]);

    fn get_num_sub_parts(&self) -> usize;

    fn has_premade_aabb(&self) -> bool {
        false
    }
    // fn set_premade_aabb(&mut self, aabb_min: Vec3A, aabb_max: Vec3A);
    fn get_premade_aabb(&self, aabb_min: &mut Vec3A, aabb_max: &mut Vec3A);
}
