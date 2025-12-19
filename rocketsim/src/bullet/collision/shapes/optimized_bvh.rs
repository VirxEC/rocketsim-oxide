use glam::Vec3A;

use super::triangle_mesh::TriangleMesh;
use crate::bullet::{
    collision::broadphase::{Bvh, BvhNode, NodeType},
    linear_math::aabb_util_2::Aabb,
};

fn update_triangle_aabb(mut aabb: Aabb) -> Aabb {
    const MIN_AABB_DIMENSION: f32 = 0.002;
    const MIN_AABB_HALF_DIMENSION: f32 = MIN_AABB_DIMENSION / 2.0;

    let diff = (aabb.max - aabb.min).cmplt(const { Vec3A::splat(MIN_AABB_DIMENSION) });

    if diff.any() {
        let [x, y, z] = diff.into();

        if x {
            aabb.max.x += MIN_AABB_HALF_DIMENSION;
            aabb.min.x -= MIN_AABB_HALF_DIMENSION;
        }

        if y {
            aabb.max.y += MIN_AABB_HALF_DIMENSION;
            aabb.min.y -= MIN_AABB_HALF_DIMENSION;
        }

        if z {
            aabb.max.z += MIN_AABB_HALF_DIMENSION;
            aabb.min.z -= MIN_AABB_HALF_DIMENSION;
        }
    }

    aabb
}

pub struct OptimizedBvh {
    pub bvh: Bvh,
}

impl OptimizedBvh {
    pub fn new(triangles: &TriangleMesh, aabb: Aabb) -> Self {
        let mut leaf_nodes: Vec<_> = triangles
            .get_tris_aabbs()
            .1
            .iter()
            .copied()
            .map(update_triangle_aabb)
            .enumerate()
            .map(|(triangle_index, aabb)| BvhNode {
                aabb,
                node_type: NodeType::Leaf { triangle_index },
            })
            .collect();
        let num_leaf_nodes = leaf_nodes.len();

        let mut bvh = Bvh {
            aabb,
            cur_node_index: 0,
            contiguous_nodes: vec![BvhNode::DEFAULT; 2 * num_leaf_nodes].into_boxed_slice(),
        };

        bvh.build_tree(&mut leaf_nodes, 0, num_leaf_nodes);

        Self { bvh }
    }
}
