use std::mem;

use glam::Vec3A;

use crate::bullet::{
    collision::shapes::{
        triangle_callback::TriangleCallback, triangle_mesh::TriangleMesh,
        triangle_shape::TriangleShape,
    },
    linear_math::{
        LARGE_FLOAT,
        aabb_util_2::{Aabb, intersect_ray_aabb, test_aabb_against_aabb},
    },
};

pub trait NodeOverlapCallback {
    fn process_node(&mut self, triangle_index: usize);
}

pub struct BvhNodeOverlapCallback<'a, T: TriangleCallback> {
    tris: &'a [TriangleShape],
    aabbs: &'a [Aabb],
    callback: &'a mut T,
}

impl<'a, T: TriangleCallback> BvhNodeOverlapCallback<'a, T> {
    pub fn new(mesh_interface: &'a TriangleMesh, callback: &'a mut T) -> Self {
        let (tris, aabbs) = mesh_interface.get_tris_aabbs();

        Self {
            tris,
            aabbs,
            callback,
        }
    }
}

impl<T: TriangleCallback> NodeOverlapCallback for BvhNodeOverlapCallback<'_, T> {
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
    lambda_max: f32,
}

#[derive(Default)]
pub struct Bvh {
    pub aabb: Aabb,
    pub cur_node_index: usize,
    pub contiguous_nodes: Box<[BvhNode]>,
}

impl Bvh {
    const SAH_BINS: usize = 12;

    fn calc_sah_split(leaf_nodes: &mut [BvhNode], start_index: usize, end_index: usize) -> usize {
        let count = end_index - start_index;
        debug_assert!(count >= 2);

        if count == 2 {
            return start_index + 1;
        }

        // Compute centroid bounds
        let mut cmin = Vec3A::splat(f32::INFINITY);
        let mut cmax = Vec3A::splat(f32::NEG_INFINITY);
        for leaf in &leaf_nodes[start_index..end_index] {
            let c = 0.5 * (leaf.aabb.min + leaf.aabb.max);
            cmin = cmin.min(c);
            cmax = cmax.max(c);
        }

        let extent = cmax - cmin;

        let mut best_axis = None;
        let mut best_cost = f32::INFINITY;
        let mut best_bin = 0;

        // Temporary storage per axis
        let mut bin_counts = [0usize; Self::SAH_BINS];
        let mut bin_bounds = [Aabb::ZERO; Self::SAH_BINS];

        for axis in 0..3 {
            if extent[axis] <= f32::EPSILON {
                continue;
            }

            // reset bins
            bin_counts.fill(0);
            bin_bounds.fill(Aabb::ZERO);

            // Fill bins
            let scale = (Self::SAH_BINS as f32 - 1.0) / extent[axis];
            for leaf in &leaf_nodes[start_index..end_index] {
                let c = 0.5 * (leaf.aabb.min + leaf.aabb.max);
                let idx = ((c[axis] - cmin[axis]) * scale).clamp(0.0, (Self::SAH_BINS - 1) as f32)
                    as usize;
                if bin_counts[idx] == 0 {
                    bin_bounds[idx] = leaf.aabb;
                } else {
                    bin_bounds[idx] += leaf.aabb;
                }
                bin_counts[idx] += 1;
            }

            // Prefix areas/counts
            let mut prefix_area = [0.0; Self::SAH_BINS];
            let mut prefix_count = [0usize; Self::SAH_BINS];
            let mut running_aabb = Aabb::ZERO;
            let mut running_count = 0usize;
            for i in 0..Self::SAH_BINS {
                if bin_counts[i] > 0 {
                    running_aabb += bin_bounds[i];
                    running_count += bin_counts[i];
                }
                prefix_area[i] = running_aabb.area();
                prefix_count[i] = running_count;
            }

            // Suffix areas/counts
            let mut suffix_area = [0.0; Self::SAH_BINS];
            let mut suffix_count = [0usize; Self::SAH_BINS];
            running_aabb = Aabb::ZERO;
            running_count = 0;
            for i in (0..Self::SAH_BINS).rev() {
                if bin_counts[i] > 0 {
                    running_aabb += bin_bounds[i];
                    running_count += bin_counts[i];
                }

                suffix_area[i] = running_aabb.area();
                suffix_count[i] = running_count;
            }

            // Evaluate splits between bins
            for i in 0..Self::SAH_BINS - 1 {
                let nl = prefix_count[i];
                let nr = suffix_count[i + 1];
                if nl == 0 || nr == 0 {
                    continue;
                }

                let cost = prefix_area[i] * nl as f32 + suffix_area[i + 1] * nr as f32;
                if cost < best_cost {
                    best_cost = cost;
                    best_axis = Some(axis);
                    best_bin = i;
                }
            }
        }

        let axis = best_axis.unwrap();

        // Partition by chosen axis/bin
        let split_value =
            cmin[axis] + extent[axis] * ((best_bin + 1) as f32) / (Self::SAH_BINS as f32);
        let mut mid = start_index;
        for i in start_index..end_index {
            let c = 0.5 * (leaf_nodes[i].aabb.min + leaf_nodes[i].aabb.max);
            if c[axis] <= split_value {
                if i != mid {
                    Self::swap_leaf_nodes(leaf_nodes, i, mid);
                }
                mid += 1;
            }
        }

        mid
    }

    fn swap_leaf_nodes(leaf_nodes: &mut [BvhNode], i: usize, split_index: usize) {
        debug_assert_ne!(i, split_index);
        let [a, b] = unsafe { leaf_nodes.get_disjoint_unchecked_mut([split_index, i]) };
        mem::swap(a, b);
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

        let split_index = Self::calc_sah_split(leaf_nodes, start_index, end_index);

        let internal_node_index = self.cur_node_index;

        {
            let node = &mut self.contiguous_nodes[internal_node_index];
            node.aabb.min = self.aabb.max;
            node.aabb.max = self.aabb.min;

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
                        && intersect_ray_aabb(
                            ray_info.source,
                            ray_info.direction_inverse,
                            &root_node.aabb,
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
        if !test_aabb_against_aabb(&aabb, &self.aabb) {
            return;
        }

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
            source: ray_source,
            direction_inverse: ray_dir_inv,
            lambda_max,
        };

        self.walk_stackless_tree_against_ray(node_callback, &ray_info, 0, self.cur_node_index);
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
