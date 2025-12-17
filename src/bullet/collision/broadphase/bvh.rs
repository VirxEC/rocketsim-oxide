use std::mem;

use glam::Vec3A;

use crate::bullet::{
    collision::shapes::{
        triangle_callback::TriangleCallback, triangle_mesh::TriangleMesh,
        triangle_shape::TriangleShape,
    },
    linear_math::{
        LARGE_FLOAT,
        aabb_util_2::{Aabb, ray_aabb_2, test_aabb_against_aabb},
    },
};

pub trait NodeOverlapCallback {
    fn process_node(&mut self, triangle_index: usize);
}

pub struct MyNodeOverlapCallback<'a, T: TriangleCallback> {
    tris: &'a [TriangleShape],
    aabbs: &'a [Aabb],
    callback: &'a mut T,
}

impl<'a, T: TriangleCallback> MyNodeOverlapCallback<'a, T> {
    pub fn new(mesh_interface: &'a TriangleMesh, callback: &'a mut T) -> Self {
        let (tris, aabbs) = mesh_interface.get_tris_aabbs();

        Self {
            tris,
            aabbs,
            callback,
        }
    }
}

impl<T: TriangleCallback> NodeOverlapCallback for MyNodeOverlapCallback<'_, T> {
    fn process_node(&mut self, node_triangle_index: usize) {
        self.callback.process_triangle(
            &self.tris[node_triangle_index],
            &self.aabbs[node_triangle_index],
            node_triangle_index,
        );
    }
}

pub struct RayInfo {
    source: Vec3A,
    aabb: Aabb,
    direction_inverse: Vec3A,
    sign: [usize; 3],
    lambda_max: f32,
}

#[derive(Default)]
pub struct Bvh {
    pub aabb: Aabb,
    pub cur_node_index: usize,
    pub contiguous_nodes: Box<[BvhNode]>,
}

impl Bvh {
    fn calc_means(leaf_nodes: &[BvhNode], start_index: usize, end_index: usize) -> Vec3A {
        let mut means = Vec3A::ZERO;
        for leaf in &leaf_nodes[start_index..end_index] {
            let center = 0.5 * (leaf.aabb.max + leaf.aabb.min);
            means += center;
        }

        let num_indices = end_index - start_index;
        means / num_indices as f32
    }

    fn calc_splitting_axis(
        leaf_nodes: &[BvhNode],
        start_index: usize,
        end_index: usize,
        means: Vec3A,
    ) -> usize {
        let mut variance = Vec3A::ZERO;
        for leaf in &leaf_nodes[start_index..end_index] {
            let center = 0.5 * (leaf.aabb.max + leaf.aabb.min);
            let diff = center - means;
            variance += diff * diff;
        }

        let num_indices = end_index - start_index;
        variance /= (num_indices - 1) as f32;
        variance.max_position()
    }

    fn swap_leaf_nodes(leaf_nodes: &mut [BvhNode], i: usize, split_index: usize) {
        debug_assert_ne!(i, split_index);
        let [a, b] = unsafe { leaf_nodes.get_disjoint_unchecked_mut([split_index, i]) };
        mem::swap(a, b);
    }

    fn sort_and_calc_splitting_index(
        leaf_nodes: &mut [BvhNode],
        start_index: usize,
        end_index: usize,
        split_axis: usize,
        means: Vec3A,
    ) -> usize {
        let mut split_index = start_index;

        let split_value = means[split_axis];
        for i in start_index..end_index {
            let center = 0.5 * (leaf_nodes[i].aabb.max + leaf_nodes[i].aabb.min);
            if center[split_axis] > split_value {
                if i != split_index {
                    Self::swap_leaf_nodes(leaf_nodes, i, split_index);
                }
                split_index += 1;
            }
        }

        let num_indices = end_index - start_index;
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

    pub fn build_tree(&mut self, leaf_nodes: &mut [BvhNode], start_index: usize, end_index: usize) {
        let num_indices = end_index - start_index;
        let cur_index = self.cur_node_index;

        debug_assert!(num_indices > 0);

        if num_indices == 1 {
            self.contiguous_nodes[self.cur_node_index] = leaf_nodes[start_index];
            self.cur_node_index += 1;
            return;
        }

        let means = Self::calc_means(leaf_nodes, start_index, end_index);
        let split_axis = Self::calc_splitting_axis(leaf_nodes, start_index, end_index, means);
        let split_index = Self::sort_and_calc_splitting_index(
            leaf_nodes,
            start_index,
            end_index,
            split_axis,
            means,
        );

        let internal_node_index = self.cur_node_index;

        {
            let node = &mut self.contiguous_nodes[internal_node_index];
            node.aabb.min = self.aabb.max;
            node.aabb.max = self.aabb.min;

            // calculate subtree aabb
            for leaf in &leaf_nodes[start_index..end_index] {
                node.aabb += leaf.aabb;
            }
        }

        self.cur_node_index += 1;

        self.build_tree(leaf_nodes, start_index, split_index);
        self.build_tree(leaf_nodes, split_index, end_index);

        let escape_index = self.cur_node_index - cur_index;
        self.contiguous_nodes[internal_node_index].node_type = NodeType::Branch { escape_index };
    }

    fn walk_stackless_tree<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        aabb: &Aabb,
        start_node_index: usize,
        end_node_index: usize,
    ) {
        let mut cur_index = start_node_index;
        while cur_index < end_node_index {
            let root_node = &self.contiguous_nodes[cur_index];
            let aabb_overlap = test_aabb_against_aabb(aabb, &root_node.aabb);

            match root_node.node_type {
                NodeType::Leaf { triangle_index } => {
                    if aabb_overlap {
                        node_callback.process_node(triangle_index);
                    }

                    cur_index += 1;
                }
                NodeType::Branch { escape_index } => {
                    cur_index += if aabb_overlap { 1 } else { escape_index };
                }
            }
        }
    }

    fn walk_stackless_tree_against_ray<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        ray_info: &RayInfo,
        start_node_index: usize,
        end_node_index: usize,
    ) {
        let mut cur_index = start_node_index;
        while cur_index < end_node_index {
            let root_node = &self.contiguous_nodes[cur_index];
            let overlap = test_aabb_against_aabb(&ray_info.aabb, &root_node.aabb);

            match root_node.node_type {
                NodeType::Leaf { triangle_index } => {
                    if overlap
                        && ray_aabb_2(
                            ray_info.source,
                            ray_info.direction_inverse,
                            &ray_info.sign,
                            &[root_node.aabb.min, root_node.aabb.max],
                            0.0,
                            ray_info.lambda_max,
                        )
                    {
                        node_callback.process_node(triangle_index);
                    }

                    cur_index += 1;
                }
                NodeType::Branch { escape_index } => {
                    cur_index += if overlap { 1 } else { escape_index };
                }
            }
        }
    }

    pub fn report_ray_overlapping_node<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        ray_source: Vec3A,
        ray_target: Vec3A,
    ) {
        let aabb = Aabb::new(ray_source.min(ray_target), ray_source.max(ray_target));
        if test_aabb_against_aabb(&aabb, &self.aabb) {
            let ray_direction = (ray_target - ray_source).normalize();
            let lambda_max = ray_direction.dot(ray_target - ray_source);

            let mut ray_dir_inv = 1.0 / ray_direction;
            // replace -inf and inf with LARGE_FLOAT
            ray_dir_inv = Vec3A::select(
                ray_dir_inv.is_finite_mask(),
                ray_dir_inv,
                const { Vec3A::splat(LARGE_FLOAT) },
            );

            let ray_info = RayInfo {
                aabb,
                sign: <[bool; 3]>::from(ray_direction.cmplt(Vec3A::ZERO)).map(usize::from),
                source: ray_source,
                direction_inverse: ray_dir_inv,
                lambda_max,
            };

            self.walk_stackless_tree_against_ray(node_callback, &ray_info, 0, self.cur_node_index);
        }
    }

    pub fn report_aabb_overlapping_node<T: NodeOverlapCallback>(
        &self,
        node_callback: &mut T,
        aabb: &Aabb,
    ) {
        if test_aabb_against_aabb(aabb, &self.aabb) {
            self.walk_stackless_tree(node_callback, aabb, 0, self.cur_node_index);
        }
    }
}

#[derive(Clone, Copy)]
pub enum NodeType {
    Leaf { triangle_index: usize },
    Branch { escape_index: usize },
}

#[derive(Clone, Copy)]
pub struct BvhNode {
    pub aabb: Aabb,
    pub node_type: NodeType,
}

impl BvhNode {
    pub const DEFAULT: Self = Self {
        aabb: Aabb::ZERO,
        node_type: NodeType::Leaf { triangle_index: 0 },
    };
}
