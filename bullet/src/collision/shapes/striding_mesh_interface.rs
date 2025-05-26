use super::triangle_callback::InternalTriangleIndexCallback;
use glam::Vec3A;

pub trait StridingMeshInterface {
    fn internal_process_all_triangles(
        &self,
        callback: &mut dyn InternalTriangleIndexCallback,
        _aabb_min: &Vec3A,
        _aabb_max: &Vec3A,
    ) {
        let mut triangle = [Vec3A::ZERO; 3];

        let mesh_scaling = self.get_scaling();
        for part in 0..self.get_num_sub_parts() {
            let (verts, ids, aabbs) = self.get_verts_ids_aabbs(part);

            for (i, (inner_ids, (aabb_min, aabb_max))) in ids.chunks_exact(3).zip(aabbs).enumerate()
            {
                for (vert, &id) in triangle.iter_mut().zip(inner_ids) {
                    *vert = verts[id] * mesh_scaling;
                }

                let continue_processing = callback
                    .internal_process_triangle_index(&triangle, *aabb_min, *aabb_max, part, i);
                if !continue_processing {
                    return;
                }
            }
        }
    }

    fn get_total_num_faces(&self) -> usize;

    fn get_verts_ids_aabbs(&self, subpart: usize) -> (&[Vec3A], &[usize], &[(Vec3A, Vec3A)]);

    fn get_num_sub_parts(&self) -> usize;

    fn has_premade_aabb(&self) -> bool {
        false
    }
    // fn set_premade_aabb(&mut self, aabb_min: Vec3A, aabb_max: Vec3A);
    fn get_premade_aabb(&self, aabb_min: &mut Vec3A, aabb_max: &mut Vec3A);

    fn get_scaling(&self) -> Vec3A;
    fn set_scaling(&mut self, scaling: Vec3A);
}
