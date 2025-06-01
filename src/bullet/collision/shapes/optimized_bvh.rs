use super::triangle_mesh::TriangleMesh;
use crate::bullet::collision::{
    broadphase::quantized_bvh::{
        BvhSubtreeInfo, MAX_NUM_PARTS_IN_BITS, NodeType, QuantizedBvh, QuantizedBvhNode,
    },
    shapes::{triangle_callback::InternalTriangleIndexCallback, triangle_shape::TriangleShape},
};
use glam::Vec3A;

struct QuantizedNodeTriangleCallback<'a> {
    pub optimized_tree: &'a mut QuantizedBvh,
}

impl InternalTriangleIndexCallback for QuantizedNodeTriangleCallback<'_> {
    fn internal_process_triangle_index(
        &mut self,
        _triangle: &TriangleShape,
        mut aabb_min: Vec3A,
        mut aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool {
        const MIN_AABB_DIMENSION: f32 = 0.002;
        const MIN_AABB_HALF_DIMENSION: f32 = MIN_AABB_DIMENSION / 2.0;

        debug_assert!(part_id < (1 << MAX_NUM_PARTS_IN_BITS));
        debug_assert!(triangle_index < (1 << (31 - MAX_NUM_PARTS_IN_BITS)));

        let diff = (aabb_max - aabb_min).cmplt(const { Vec3A::splat(MIN_AABB_DIMENSION) });

        if diff.any() {
            let [x, y, z] = diff.into();

            if x {
                aabb_max.x += MIN_AABB_HALF_DIMENSION;
                aabb_min.x -= MIN_AABB_HALF_DIMENSION;
            }

            if y {
                aabb_max.y += MIN_AABB_HALF_DIMENSION;
                aabb_min.y -= MIN_AABB_HALF_DIMENSION;
            }

            if z {
                aabb_max.z += MIN_AABB_HALF_DIMENSION;
                aabb_min.z -= MIN_AABB_HALF_DIMENSION;
            }
        }

        let node = QuantizedBvhNode {
            quantized_aabb_min: self.optimized_tree.quantize(aabb_min, false),
            quantized_aabb_max: self.optimized_tree.quantize(aabb_max, true),
            node_type: NodeType::Leaf {
                part_id,
                triangle_index,
            },
        };

        self.optimized_tree.quantized_leaf_nodes.push(node);
        true
    }
}

pub struct OptimizedBvh {
    pub quantized_bvh: QuantizedBvh,
}

impl OptimizedBvh {
    pub fn new(triangles: &TriangleMesh, local_aabb_min: Vec3A, local_aabb_max: Vec3A) -> Self {
        let mut quantized_bvh = QuantizedBvh::default();

        quantized_bvh.set_quantization_values(local_aabb_min, local_aabb_max);
        quantized_bvh
            .quantized_leaf_nodes
            .reserve(triangles.get_total_num_faces());

        let min_aabb = quantized_bvh.bvh_aabb_min;
        let max_aabb = quantized_bvh.bvh_aabb_max;

        let mut callback = QuantizedNodeTriangleCallback {
            optimized_tree: &mut quantized_bvh,
        };

        triangles.internal_process_all_triangles(&mut callback, &min_aabb, &max_aabb);

        let num_leaf_nodes = quantized_bvh.quantized_leaf_nodes.len();
        quantized_bvh
            .quantized_contiguous_nodes
            .resize(2 * num_leaf_nodes, QuantizedBvhNode::DEFAULT);

        quantized_bvh.cur_node_index = 0;
        quantized_bvh.build_tree(0, num_leaf_nodes);

        if quantized_bvh.subtree_headers.is_empty() {
            quantized_bvh.subtree_headers.push(BvhSubtreeInfo {
                quantized_aabb_min: quantized_bvh.quantized_contiguous_nodes[0].quantized_aabb_min,
                quantized_aabb_max: quantized_bvh.quantized_contiguous_nodes[0].quantized_aabb_max,
                root_node_index: 0,
                subtree_size: quantized_bvh.quantized_contiguous_nodes[0].get_subtree_size(),
            });
        }

        quantized_bvh.quantized_leaf_nodes.clear();

        Self { quantized_bvh }
    }
}
