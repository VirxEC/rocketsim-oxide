use super::{
    striding_mesh_interface::StridingMeshInterface,
    triangle_index_vertex_array::{IndexedMesh, TriangleIndexVertexArray},
};
use glam::Vec3A;
use std::sync::Arc;

#[derive(Debug)]
pub struct TriangleMesh {
    pub(crate) triangle_index_vertex_array: TriangleIndexVertexArray,
    pub(crate) four_component_vertices: Vec<Vec3A>,
    pub(crate) u32_indices: Vec<u32>,
    // pub(crate) welding_threshold: f32,
}

impl Default for TriangleMesh {
    fn default() -> Self {
        let indexed_mesh = IndexedMesh {
            triangle_index_stride: 3 * size_of::<i32>(),
            vertex_stride: size_of::<Vec3A>(),
            ..Default::default()
        };

        Self {
            four_component_vertices: Vec::new(),
            triangle_index_vertex_array: TriangleIndexVertexArray {
                indexed_meshes: vec![indexed_mesh],
                ..Default::default()
            },
            u32_indices: Vec::new(),
            // welding_threshold: 0.0,
        }
    }
}

impl TriangleMesh {
    // fn find_or_add_vertex(&mut self, vertex: Vec3A, remove_duplicate_vertices: bool) -> usize {
    //     if remove_duplicate_vertices {
    //         unimplemented!();
    //     }

    //     self.triangle_index_vertex_array.indexed_meshes[0].num_vertices += 1;
    //     self.four_component_vertices.push(vertex);

    //     let ptr = &self.four_component_vertices[0] as *const _ as *const u8;
    //     self.triangle_index_vertex_array.indexed_meshes[0].vertex_base = Some(ptr);

    //     self.four_component_vertices.len() - 1
    // }

    pub fn set_vertices(&mut self, vertices: Vec<Vec3A>) {
        self.triangle_index_vertex_array.indexed_meshes[0].num_vertices = vertices.len();
        self.four_component_vertices = vertices;

        let ptr = self.four_component_vertices.as_ptr();
        self.triangle_index_vertex_array.indexed_meshes[0].vertex_base = Some(ptr.cast::<u8>());
    }

    // fn add_index(&mut self, index: usize) {
    //     self.u32_indices.push(index as u32);

    //     let ptr = &self.u32_indices[0] as *const _ as *const u8;
    //     self.triangle_index_vertex_array.indexed_meshes[0].triangle_index_base = Some(ptr);
    // }

    // fn add_triangle_indices(&mut self, index1: usize, index2: usize, index3: usize) {
    //     self.triangle_index_vertex_array.indexed_meshes[0].num_triangles += 1;

    //     self.add_index(index1);
    //     self.add_index(index2);
    //     self.add_index(index3);
    // }

    pub fn set_indices(&mut self, indices: Vec<u32>) {
        debug_assert_eq!(indices.len() % 3, 0);

        self.triangle_index_vertex_array.indexed_meshes[0].num_triangles += indices.len() / 3;
        self.u32_indices = indices;

        let ptr = self.u32_indices.as_ptr();
        self.triangle_index_vertex_array.indexed_meshes[0].triangle_index_base =
            Some(ptr.cast::<u8>());
    }

    pub fn into_mesh_interface(self) -> Arc<dyn StridingMeshInterface + Send + Sync> {
        Arc::new(self)
    }
}

impl StridingMeshInterface for TriangleMesh {
    fn get_num_sub_parts(&self) -> usize {
        self.triangle_index_vertex_array.get_num_sub_parts()
    }

    fn get_premade_aabb(&self, aabb_min: &mut Vec3A, aabb_max: &mut Vec3A) {
        self.triangle_index_vertex_array
            .get_premade_aabb(aabb_min, aabb_max);
    }

    fn has_premade_aabb(&self) -> bool {
        self.triangle_index_vertex_array.has_premade_aabb()
    }

    // fn set_premade_aabb(&mut self, aabb_min: Vec3A, aabb_max: Vec3A) {
    //     self.triangle_index_vertex_array
    //         .set_premade_aabb(aabb_min, aabb_max);
    // }

    fn get_total_num_faces(&self) -> usize {
        self.triangle_index_vertex_array.get_total_num_faces()
    }

    fn get_scaling(&self) -> Vec3A {
        self.triangle_index_vertex_array.get_scaling()
    }

    fn set_scaling(&mut self, scaling: Vec3A) {
        self.triangle_index_vertex_array.set_scaling(scaling);
    }

    fn get_locked_read_only_vertex_index_base(
        &self,
        vertex_base: &mut *const u8,
        num_verts: &mut usize,
        vertex_stride: &mut usize,
        index_base: &mut *const u8,
        index_stride: &mut usize,
        num_faces: &mut usize,
        subpart: usize,
    ) {
        self.triangle_index_vertex_array
            .get_locked_read_only_vertex_index_base(
                vertex_base,
                num_verts,
                vertex_stride,
                index_base,
                index_stride,
                num_faces,
                subpart,
            );
    }

    fn unlock_read_only_vertex_base(&self, subpart: usize) {
        self.triangle_index_vertex_array
            .unlock_read_only_vertex_base(subpart);
    }
}
