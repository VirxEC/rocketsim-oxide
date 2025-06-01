use crate::{
    collision::shapes::{triangle_callback::TriangleCallback, triangle_mesh::TriangleMesh},
    linear_math::aabb_util_2::test_quantized_aabb_against_quantized_aabb,
};
use glam::{U16Vec3, Vec3A};
use std::mem;

pub const MAX_SUBTREE_SIZE_IN_BYTES: usize = 2048;
pub const MAX_NUM_PARTS_IN_BITS: usize = 10;

pub trait NodeOverlapCallback {
    fn process_node(&mut self, subpart: usize, triangle_index: usize);
}

pub struct MyNodeOverlapCallback<'a, T: TriangleCallback> {
    mesh_interface: &'a TriangleMesh,
    callback: &'a mut T,
    num_overlap: usize,
}

impl<'a, T: TriangleCallback> MyNodeOverlapCallback<'a, T> {
    pub fn new(mesh_interface: &'a TriangleMesh, callback: &'a mut T) -> Self {
        Self {
            mesh_interface,
            callback,
            num_overlap: 0,
        }
    }
}

impl<T: TriangleCallback> NodeOverlapCallback for MyNodeOverlapCallback<'_, T> {
    fn process_node(&mut self, node_subpart: usize, node_triangle_index: usize) {
        self.num_overlap += 1;

        let (tris, aabbs) = self.mesh_interface.get_tris_aabbs(node_subpart);
        let (aabb_min, aabb_max) = aabbs[node_triangle_index];

        self.callback.process_triangle(
            &tris[node_triangle_index],
            aabb_min,
            aabb_max,
            node_subpart,
            node_triangle_index,
        );
    }
}

#[derive(Default)]
pub struct QuantizedBvh {
    pub bvh_aabb_min: Vec3A,
    pub bvh_aabb_max: Vec3A,
    pub bvh_quantization: Vec3A,
    pub cur_node_index: usize,
    pub quantized_leaf_nodes: Vec<QuantizedBvhNode>,
    pub quantized_contiguous_nodes: Vec<QuantizedBvhNode>,
    pub subtree_headers: Vec<BvhSubtreeInfo>,
}

impl QuantizedBvh {
    pub fn quantize(&self, point: Vec3A, is_max: bool) -> U16Vec3 {
        // dbg!(point, self.bvh_aabb_max, self.bvh_aabb_min);
        debug_assert!(point.x <= self.bvh_aabb_max.x);
        debug_assert!(point.y <= self.bvh_aabb_max.y);
        debug_assert!(point.z <= self.bvh_aabb_max.z);

        debug_assert!(point.x >= self.bvh_aabb_min.x);
        debug_assert!(point.y >= self.bvh_aabb_min.y);
        debug_assert!(point.z >= self.bvh_aabb_min.z);

        let v = (point - self.bvh_aabb_min) * self.bvh_quantization;

        if is_max {
            (v + 1.0).as_u16vec3() | 1
        } else {
            v.as_u16vec3() & 0xfffe
        }
    }

    pub fn quantize_with_clamp(&self, point: Vec3A, is_max: bool) -> U16Vec3 {
        let clamped_point = point.clamp(self.bvh_aabb_min, self.bvh_aabb_max);
        self.quantize(clamped_point, is_max)
    }

    pub fn unquantize(&self, vec_in: U16Vec3) -> Vec3A {
        vec_in.as_vec3a() / self.bvh_quantization + self.bvh_aabb_min
    }

    pub fn set_quantization_values(&mut self, bvh_aabb_min: Vec3A, bvh_aabb_max: Vec3A) {
        let quantization_margin = 1.0;
        let clamp_value = Vec3A::splat(quantization_margin);

        self.bvh_aabb_min = bvh_aabb_min - clamp_value;
        self.bvh_aabb_max = bvh_aabb_max + clamp_value;

        let mut aabb_size = self.bvh_aabb_max - self.bvh_aabb_min;
        self.bvh_quantization = Vec3A::splat(65533.0) / aabb_size;

        {
            let vec_in = self.quantize(self.bvh_aabb_min, false);
            let v = self.unquantize(vec_in);
            self.bvh_aabb_min = self.bvh_aabb_min.min(v - clamp_value);
        }
        aabb_size = self.bvh_aabb_max - self.bvh_aabb_min;
        self.bvh_quantization = Vec3A::splat(65533.0) / aabb_size;

        {
            let vec_in = self.quantize(self.bvh_aabb_max, true);
            let v = self.unquantize(vec_in);
            self.bvh_aabb_max = self.bvh_aabb_max.max(v + clamp_value);
        }
        aabb_size = self.bvh_aabb_max - self.bvh_aabb_min;
        self.bvh_quantization = Vec3A::splat(65533.0) / aabb_size;
    }

    fn get_aabb_max(&self, node_index: usize) -> Vec3A {
        self.unquantize(self.quantized_leaf_nodes[node_index].quantized_aabb_max)
    }

    fn get_aabb_min(&self, node_index: usize) -> Vec3A {
        self.unquantize(self.quantized_leaf_nodes[node_index].quantized_aabb_min)
    }

    fn calc_splitting_axis(&self, start_index: usize, end_index: usize) -> usize {
        let num_indices = end_index - start_index;

        let mut means = Vec3A::ZERO;
        for i in start_index..end_index {
            let center = 0.5 * (self.get_aabb_max(i) + self.get_aabb_min(i));
            means += center;
        }
        means *= 1.0 / num_indices as f32;

        let mut variance = Vec3A::ZERO;
        for i in start_index..end_index {
            let center = 0.5 * (self.get_aabb_max(i) + self.get_aabb_min(i));
            let diff = center - means;
            variance += diff * diff;
        }
        variance *= 1.0 / (num_indices - 1) as f32;

        variance.max_position()
    }

    fn swap_leaf_nodes(&mut self, i: usize, split_index: usize) {
        debug_assert_ne!(i, split_index);
        let [a, b] = unsafe {
            self.quantized_leaf_nodes
                .get_disjoint_unchecked_mut([split_index, i])
        };

        mem::swap(a, b);
    }

    fn sort_and_calc_splitting_index(
        &mut self,
        start_index: usize,
        end_index: usize,
        split_axis: usize,
    ) -> usize {
        let mut split_index = start_index;
        let num_indices = end_index - start_index;

        let mut means = Vec3A::ZERO;
        for i in start_index..end_index {
            let center = 0.5 * (self.get_aabb_max(i) + self.get_aabb_min(i));
            means += center;
        }
        means *= 1.0 / num_indices as f32;

        let split_value = means[split_axis];
        for i in start_index..end_index {
            let center = 0.5 * (self.get_aabb_max(i) + self.get_aabb_min(i));
            if center[split_axis] > split_value {
                if i != split_index {
                    self.swap_leaf_nodes(i, split_index);
                }
                split_index += 1;
            }
        }

        let range_balanced_indices = num_indices / 3;
        let unbalanced = split_index <= start_index + range_balanced_indices
            || split_index >= end_index - 1 - range_balanced_indices;

        if unbalanced {
            split_index = start_index + (num_indices >> 1);
        }

        debug_assert_ne!(split_index, start_index, "tree is unbalanced");
        debug_assert_ne!(split_index, end_index, "tree is unbalanced");

        split_index
    }

    fn set_internal_node_aabb_min(&mut self, node_index: usize, aabb_min: Vec3A) {
        self.quantized_contiguous_nodes[node_index].quantized_aabb_min =
            self.quantize(aabb_min, false);
    }

    fn set_internal_node_aabb_max(&mut self, node_index: usize, aabb_max: Vec3A) {
        self.quantized_contiguous_nodes[node_index].quantized_aabb_max =
            self.quantize(aabb_max, true);
    }

    fn merge_internal_node_aabb(
        &mut self,
        node_index: usize,
        new_aabb_min: Vec3A,
        new_aabb_max: Vec3A,
    ) {
        let quantized_aabb_min = self.quantize(new_aabb_min, false);
        let quantized_aabb_max = self.quantize(new_aabb_max, true);

        for i in 0..3 {
            if self.quantized_contiguous_nodes[node_index].quantized_aabb_min[i]
                > quantized_aabb_min[i]
            {
                self.quantized_contiguous_nodes[node_index].quantized_aabb_min[i] =
                    quantized_aabb_min[i];
            }

            if self.quantized_contiguous_nodes[node_index].quantized_aabb_max[i]
                < quantized_aabb_max[i]
            {
                self.quantized_contiguous_nodes[node_index].quantized_aabb_max[i] =
                    quantized_aabb_max[i];
            }
        }
    }

    fn assign_internal_node_from_leaf_node(
        &mut self,
        internal_node: usize,
        leaf_node_index: usize,
    ) {
        self.quantized_contiguous_nodes[internal_node] = self.quantized_leaf_nodes[leaf_node_index];
    }

    fn set_internal_node_escape_index(&mut self, node_index: usize, escape_index: usize) {
        self.quantized_contiguous_nodes[node_index].node_type = NodeType::Branch { escape_index };
    }

    fn update_subtree_headers(
        &mut self,
        left_child_node_index: usize,
        right_child_node_index: usize,
    ) {
        let left_child_node = self.quantized_contiguous_nodes[left_child_node_index];
        let left_subtree_size = left_child_node.get_subtree_size();
        let left_subtree_size_in_bytes = left_subtree_size * size_of::<QuantizedBvhNode>();

        let right_child_node = self.quantized_contiguous_nodes[right_child_node_index];
        let right_subtree_size = right_child_node.get_subtree_size();
        let right_subtree_size_in_bytes = right_subtree_size * size_of::<QuantizedBvhNode>();

        if left_subtree_size_in_bytes <= MAX_SUBTREE_SIZE_IN_BYTES {
            self.subtree_headers.push(BvhSubtreeInfo {
                quantized_aabb_min: left_child_node.quantized_aabb_min,
                quantized_aabb_max: left_child_node.quantized_aabb_max,
                root_node_index: left_child_node_index,
                subtree_size: left_subtree_size,
            });
        }

        if right_subtree_size_in_bytes <= MAX_SUBTREE_SIZE_IN_BYTES {
            self.subtree_headers.push(BvhSubtreeInfo {
                quantized_aabb_min: right_child_node.quantized_aabb_min,
                quantized_aabb_max: right_child_node.quantized_aabb_max,
                root_node_index: right_child_node_index,
                subtree_size: right_subtree_size,
            });
        }
    }

    pub fn build_tree(&mut self, start_index: usize, end_index: usize) {
        let num_indices = end_index - start_index;
        let cur_index = self.cur_node_index;

        debug_assert!(num_indices > 0);

        if num_indices == 1 {
            self.assign_internal_node_from_leaf_node(self.cur_node_index, start_index);
            self.cur_node_index += 1;
            return;
        }

        let split_axis = self.calc_splitting_axis(start_index, end_index);
        let split_index = self.sort_and_calc_splitting_index(start_index, end_index, split_axis);

        let internal_node_index = self.cur_node_index;

        self.set_internal_node_aabb_min(self.cur_node_index, self.bvh_aabb_max);
        self.set_internal_node_aabb_max(self.cur_node_index, self.bvh_aabb_min);

        for i in start_index..end_index {
            self.merge_internal_node_aabb(
                self.cur_node_index,
                self.get_aabb_min(i),
                self.get_aabb_max(i),
            );
        }

        self.cur_node_index += 1;

        let left_child_node_index = self.cur_node_index;
        self.build_tree(start_index, split_index);

        let right_child_node_index = self.cur_node_index;
        self.build_tree(split_index, end_index);

        let escape_index = self.cur_node_index - cur_index;

        let tree_size_in_bytes = escape_index * size_of::<QuantizedBvhNode>();
        if tree_size_in_bytes > MAX_SUBTREE_SIZE_IN_BYTES {
            self.update_subtree_headers(left_child_node_index, right_child_node_index);
        }

        self.set_internal_node_escape_index(internal_node_index, escape_index);
    }

    fn walk_stackless_quantized_tree<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        quantized_query_aabb_min: U16Vec3,
        quantized_query_aabb_max: U16Vec3,
        start_node_index: usize,
        end_node_index: usize,
    ) {
        let mut cur_index = start_node_index;
        let mut walk_iterations = 0;
        let sub_tree_size = end_node_index - start_node_index;

        while cur_index < end_node_index {
            debug_assert!(walk_iterations < sub_tree_size);

            let root_node = &self.quantized_contiguous_nodes[cur_index];

            walk_iterations += 1;
            let aabb_overlap = test_quantized_aabb_against_quantized_aabb(
                quantized_query_aabb_min,
                quantized_query_aabb_max,
                root_node.quantized_aabb_min,
                root_node.quantized_aabb_max,
            );

            match root_node.node_type {
                NodeType::Leaf {
                    part_id,
                    triangle_index,
                } => {
                    node_callback.process_node(part_id, triangle_index);
                    cur_index += 1;
                }
                NodeType::Branch { escape_index } if !aabb_overlap => {
                    cur_index += escape_index;
                }
                _ => cur_index += 1,
            }
        }
    }

    fn walk_stackless_quantized_tree_cache_friendly<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        quantized_query_aabb_min: U16Vec3,
        quantized_query_aabb_max: U16Vec3,
    ) {
        for subtree in &self.subtree_headers {
            let overlap = test_quantized_aabb_against_quantized_aabb(
                quantized_query_aabb_min,
                quantized_query_aabb_max,
                subtree.quantized_aabb_min,
                subtree.quantized_aabb_max,
            );
            if overlap {
                self.walk_stackless_quantized_tree(
                    node_callback,
                    quantized_query_aabb_min,
                    quantized_query_aabb_max,
                    subtree.root_node_index,
                    subtree.root_node_index + subtree.subtree_size,
                );
            }
        }
    }

    pub fn report_aabb_overlapping_node<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
    ) {
        let quantized_query_aabb_min = self.quantize_with_clamp(aabb_min, false);
        let quantized_query_aabb_max = self.quantize_with_clamp(aabb_max, true);

        self.walk_stackless_quantized_tree_cache_friendly(
            node_callback,
            quantized_query_aabb_min,
            quantized_query_aabb_max,
        );
    }
}

#[derive(Clone, Copy)]
pub enum NodeType {
    Leaf {
        part_id: usize,
        triangle_index: usize,
    },
    Branch {
        escape_index: usize,
    },
}

#[derive(Clone, Copy)]
pub struct QuantizedBvhNode {
    pub quantized_aabb_min: U16Vec3,
    pub quantized_aabb_max: U16Vec3,
    pub node_type: NodeType,
}

impl QuantizedBvhNode {
    pub const DEFAULT: Self = Self {
        quantized_aabb_min: U16Vec3::ZERO,
        quantized_aabb_max: U16Vec3::ZERO,
        node_type: NodeType::Leaf {
            triangle_index: 0,
            part_id: 0,
        },
    };

    pub const fn is_leaf_node(&self) -> bool {
        matches!(
            self.node_type,
            NodeType::Leaf {
                part_id: _,
                triangle_index: _
            }
        )
    }

    pub fn get_subtree_size(&self) -> usize {
        match self.node_type {
            NodeType::Leaf {
                part_id: _,
                triangle_index: _,
            } => 1,
            NodeType::Branch { escape_index } => escape_index,
        }
    }
}

#[derive(Clone, Copy)]
pub struct BvhSubtreeInfo {
    pub quantized_aabb_min: U16Vec3,
    pub quantized_aabb_max: U16Vec3,
    pub root_node_index: usize,
    pub subtree_size: usize,
}
