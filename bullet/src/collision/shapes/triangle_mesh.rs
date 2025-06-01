use super::{triangle_callback::InternalTriangleIndexCallback, triangle_shape::TriangleShape};
use glam::Vec3A;

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

    pub fn internal_process_all_triangles<T: InternalTriangleIndexCallback>(
        &self,
        callback: &mut T,
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

    pub fn get_triangle(&self, subpart: usize, index: usize) -> TriangleShape {
        self.get_tris_aabbs(subpart).0[index]
    }

    pub fn get_num_sub_parts(&self) -> usize {
        1
    }

    pub fn get_total_num_faces(&self) -> usize {
        self.triangles.len()
    }

    pub fn get_tris_aabbs(&self, _subpart: usize) -> (&[TriangleShape], &[(Vec3A, Vec3A)]) {
        (&self.triangles, &self.aabbs)
    }
}
