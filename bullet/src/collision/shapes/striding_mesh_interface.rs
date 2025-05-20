use std::ptr;

use glam::Vec3A;

use super::triangle_callback::InternalTriangleIndexCallback;

pub trait StridingMeshInterface {
    fn internal_process_all_triangles(
        &self,
        callback: &mut dyn InternalTriangleIndexCallback,
        _aabb_min: &Vec3A,
        _aabb_max: &Vec3A,
    ) {
        let graphics_sub_parts = self.get_num_sub_parts();

        let mut vertex_base: *const u8 = ptr::null();
        let mut index_base: *const u8 = ptr::null();

        let mut index_stride = 0;
        let mut stride = 0;
        let mut num_verts = 0;
        let mut num_triangles = 0;
        let mut triangle = [Vec3A::ZERO; 3];

        let mesh_scaling = self.get_scaling();
        for part in 0..graphics_sub_parts {
            self.get_locked_read_only_vertex_index_base(
                &mut vertex_base,
                &mut num_verts,
                &mut stride,
                &mut index_base,
                &mut index_stride,
                &mut num_triangles,
                part,
            );

            for gfx_index in 0..num_triangles {
                let tri_indices =
                    unsafe { index_base.byte_add(gfx_index * index_stride) }.cast::<u32>();

                for (i, vert) in triangle.iter_mut().enumerate() {
                    let graphics_base =
                        unsafe { vertex_base.byte_add(*tri_indices.add(i) as usize * stride) }
                            .cast::<f32>();

                    vert.x = unsafe { *graphics_base.add(0) };
                    vert.y = unsafe { *graphics_base.add(1) };
                    vert.z = unsafe { *graphics_base.add(2) };
                    *vert *= mesh_scaling;
                }

                callback.internal_process_triangle_index(&triangle, part, gfx_index);
            }

            self.unlock_read_only_vertex_base(part);
        }
    }

    fn get_total_num_faces(&self) -> usize;

    fn get_locked_read_only_vertex_index_base(
        &self,
        vertex_base: &mut *const u8,
        num_verts: &mut usize,
        vertex_stride: &mut usize,
        index_base: &mut *const u8,
        index_stride: &mut usize,
        num_faces: &mut usize,
        subpart: usize,
    );

    fn unlock_read_only_vertex_base(&self, subpart: usize);

    fn get_num_sub_parts(&self) -> usize;

    fn has_premade_aabb(&self) -> bool {
        false
    }
    fn set_premade_aabb(&mut self, aabb_min: Vec3A, aabb_max: Vec3A);
    fn get_premade_aabb(&self, aabb_min: &mut Vec3A, aabb_max: &mut Vec3A);

    fn get_scaling(&self) -> Vec3A;
    fn set_scaling(&mut self, scaling: Vec3A);
}
