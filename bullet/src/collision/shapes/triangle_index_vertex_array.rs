use super::striding_mesh_interface::StridingMeshInterface;
use glam::Vec3A;

#[derive(Debug, Default)]
pub struct IndexedMesh {
    pub num_triangles: usize,
    pub triangle_index_base: Option<*const u8>,
    pub triangle_index_stride: usize,
    pub num_vertices: usize,
    pub vertex_base: Option<*const u8>,
    pub vertex_stride: usize,
}

unsafe impl Send for IndexedMesh {}
unsafe impl Sync for IndexedMesh {}

#[derive(Debug)]
pub struct TriangleIndexVertexArray {
    pub indexed_meshes: Vec<IndexedMesh>,
    pub has_aabb: bool,
    pub aabb_min: Vec3A,
    pub aabb_max: Vec3A,
    pub scaling: Vec3A,
}

impl Default for TriangleIndexVertexArray {
    fn default() -> Self {
        Self {
            indexed_meshes: Vec::new(),
            has_aabb: false,
            aabb_min: Vec3A::ZERO,
            aabb_max: Vec3A::ZERO,
            scaling: Vec3A::ONE,
        }
    }
}

impl StridingMeshInterface for TriangleIndexVertexArray {
    fn get_num_sub_parts(&self) -> usize {
        self.indexed_meshes.len()
    }

    fn has_premade_aabb(&self) -> bool {
        self.has_aabb
    }

    // fn set_premade_aabb(&mut self, aabb_min: Vec3A, aabb_max: Vec3A) {
    //     *self.aabb_min.borrow_mut() = aabb_min;
    //     *self.aabb_max.borrow_mut() = aabb_max;
    // }

    fn get_premade_aabb(&self, aabb_min: &mut Vec3A, aabb_max: &mut Vec3A) {
        *aabb_min = self.aabb_min;
        *aabb_max = self.aabb_max;
    }

    fn get_scaling(&self) -> Vec3A {
        self.scaling
    }

    fn set_scaling(&mut self, scaling: Vec3A) {
        self.scaling = scaling;
    }

    fn get_total_num_faces(&self) -> usize {
        self.indexed_meshes
            .iter()
            .map(|mesh| mesh.num_triangles)
            .sum()
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
        let mesh = &self.indexed_meshes[subpart];

        *num_verts = mesh.num_vertices;
        *vertex_base = mesh.vertex_base.unwrap();
        *vertex_stride = mesh.vertex_stride;

        *num_faces = mesh.num_triangles;
        *index_base = mesh.triangle_index_base.unwrap();
        *index_stride = mesh.triangle_index_stride;
    }

    fn unlock_read_only_vertex_base(&self, _subpart: usize) {}
}
