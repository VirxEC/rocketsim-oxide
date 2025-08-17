use super::triangle_shape::TriangleShape;
use crate::bullet::{
    collision::shapes::triangle_callback::TriangleCallback, linear_math::aabb_util_2::Aabb,
};
use glam::Vec3A;

#[derive(Debug)]
pub struct TriangleMesh {
    triangles: Box<[TriangleShape]>,
    aabbs: Box<[Aabb]>,
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

                Aabb {
                    min: p0.min(p1).min(p2),
                    max: p0.max(p1).max(p2),
                }
            })
            .collect();

        let triangles = (0..ids.len() / 3)
            .map(|i| i * 3)
            .map(|i| TriangleShape::from_points_iter(ids[i..i + 3].iter().map(|&j| verts[j])))
            .collect();

        Self { triangles, aabbs }
    }

    pub fn internal_process_all_triangles<T: TriangleCallback>(&self, callback: &mut T) {
        let (tris, aabbs) = self.get_tris_aabbs();

        for (i, (triangle, aabb)) in tris.iter().zip(aabbs).enumerate() {
            let continue_processing = callback.process_triangle(triangle, aabb, i);
            if !continue_processing {
                return;
            }
        }
    }

    pub fn get_triangle(&self, index: usize) -> &TriangleShape {
        &self.triangles[index]
    }

    pub fn get_total_num_faces(&self) -> usize {
        self.triangles.len()
    }

    pub fn get_tris_aabbs(&self) -> (&[TriangleShape], &[Aabb]) {
        (&self.triangles, &self.aabbs)
    }
}
