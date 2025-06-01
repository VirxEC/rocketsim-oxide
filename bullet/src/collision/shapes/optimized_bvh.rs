use super::striding_mesh_interface::StridingMeshInterface;
use crate::collision::{
    broadphase::quantized_bvh::{
        BvhSubtreeInfo, MAX_NUM_PARTS_IN_BITS, QuantizedBvh, QuantizedBvhNode,
    },
    shapes::{triangle_callback::InternalTriangleIndexCallback, triangle_shape::TriangleShape},
};
use glam::Vec3A;

#[derive(Clone, Default)]
pub struct OptimizedBvh {
    pub quantized_bvh: QuantizedBvh,
}

impl OptimizedBvh {
    pub fn build(
        &mut self,
        triangles: &dyn StridingMeshInterface,
        local_aabb_min: Vec3A,
        local_aabb_max: Vec3A,
    ) {
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
                    escape_index_or_triangle_index: ((part_id << (31 - MAX_NUM_PARTS_IN_BITS))
                        | triangle_index)
                        as i32,
                };

                self.optimized_tree.quantized_leaf_nodes.push(node);
                true
            }
        }

        self.quantized_bvh
            .set_quantization_values(local_aabb_min, local_aabb_max);
        self.quantized_bvh
            .quantized_leaf_nodes
            .reserve(triangles.get_total_num_faces());

        let min_aabb = self.quantized_bvh.bvh_aabb_min;
        let max_aabb = self.quantized_bvh.bvh_aabb_max;

        let mut callback = QuantizedNodeTriangleCallback {
            optimized_tree: &mut self.quantized_bvh,
        };

        triangles.internal_process_all_triangles(&mut callback, &min_aabb, &max_aabb);

        let num_leaf_nodes = self.quantized_bvh.quantized_leaf_nodes.len();
        self.quantized_bvh
            .quantized_contiguous_nodes
            .resize(2 * num_leaf_nodes, QuantizedBvhNode::DEFAULT);

        self.quantized_bvh.cur_node_index = 0;
        self.quantized_bvh.build_tree(0, num_leaf_nodes);

        if self.quantized_bvh.subtree_headers.is_empty() {
            self.quantized_bvh.subtree_headers.push(BvhSubtreeInfo {
                quantized_aabb_min: self.quantized_bvh.quantized_contiguous_nodes[0]
                    .quantized_aabb_min,
                quantized_aabb_max: self.quantized_bvh.quantized_contiguous_nodes[0]
                    .quantized_aabb_max,
                root_node_index: 0,
                subtree_size: if self.quantized_bvh.quantized_contiguous_nodes[0].is_leaf_node() {
                    1
                } else {
                    self.quantized_bvh.quantized_contiguous_nodes[0].get_escape_index()
                },
            });
        }

        self.quantized_bvh.quantized_leaf_nodes.clear();
    }
}
