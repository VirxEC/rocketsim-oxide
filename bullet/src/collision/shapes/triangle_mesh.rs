use super::{striding_mesh_interface::StridingMeshInterface, triangle_shape::TriangleShape};
use glam::Vec3A;
use std::sync::Arc;

#[derive(Debug)]
pub struct TriangleMesh {
    triangles: Box<[TriangleShape]>,
    aabbs: Box<[(Vec3A, Vec3A)]>,
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

        let triangles = (0..ids.len() / 3)
            .map(|i| i * 3)
            .map(|i| TriangleShape::from_points_iter(ids[i..i + 3].iter().map(|&j| verts[j])))
            .collect();

        Self { triangles, aabbs }
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
        self.triangles.len()
    }

    fn get_tris_aabbs(&self, _subpart: usize) -> (&[TriangleShape], &[(Vec3A, Vec3A)]) {
        (&self.triangles, &self.aabbs)
    }
}
