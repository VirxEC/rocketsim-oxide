use super::striding_mesh_interface::StridingMeshInterface;
use glam::Vec3A;
use std::sync::Arc;

#[derive(Debug)]
pub struct TriangleMesh {
    verts: Box<[Vec3A]>,
    ids: Box<[usize]>,
    aabbs: Box<[(Vec3A, Vec3A)]>,
    scaling: Vec3A,
}

impl TriangleMesh {
    #[must_use]
    pub fn new(verts: Vec<Vec3A>, ids: Vec<usize>) -> Self {
        debug_assert_eq!(ids.len() % 3, 0);

        let aabbs = ids
            .chunks_exact(3)
            .map(|ids| {
                let p0 = verts[ids[0]];
                let p1 = verts[ids[1]];
                let p2 = verts[ids[2]];

                (p0.min(p1).min(p2), p0.max(p1).max(p2))
            })
            .collect();

        Self {
            verts: verts.into_boxed_slice(),
            ids: ids.into_boxed_slice(),
            aabbs,
            scaling: Vec3A::ONE,
        }
    }

    #[must_use]
    pub fn into_mesh_interface(self) -> Arc<dyn StridingMeshInterface + Send + Sync> {
        Arc::new(self)
    }
}

impl StridingMeshInterface for TriangleMesh {
    fn get_num_sub_parts(&self) -> usize {
        1
    }

    fn get_premade_aabb(&self, _aabb_min: &mut Vec3A, _aabb_max: &mut Vec3A) {
        unimplemented!()
    }

    fn has_premade_aabb(&self) -> bool {
        false
    }

    fn get_total_num_faces(&self) -> usize {
        self.ids.len() / 3
    }

    fn get_scaling(&self) -> Vec3A {
        self.scaling
    }

    fn set_scaling(&mut self, scaling: Vec3A) {
        self.scaling = scaling;
    }

    fn get_verts_ids_aabbs(&self, _subpart: usize) -> (&[Vec3A], &[usize], &[(Vec3A, Vec3A)]) {
        (&self.verts, &self.ids, &self.aabbs)
    }
}
