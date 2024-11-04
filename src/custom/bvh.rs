use glam::Vec3A;

use crate::bullet::linear_math::transform::Transform;

use super::{
    geometry::{Aabb, Tri},
    morton::Morton,
};
use std::{boxed::Box, convert::Into};

/// A leaf in the BVH.
#[derive(Clone, Copy, Debug)]
pub struct Leaf {
    /// The bounding box of this leaf.
    pub aabb: Aabb,
    /// The primitive that this leaf represents.
    pub primitive: Tri,
    /// The morton code of this leaf.
    pub morton: u64,
}

impl Leaf {
    #[must_use]
    #[inline]
    /// Create a new leaf.
    pub const fn new(primitive: Tri, box_: Aabb, morton: u64) -> Self {
        Self {
            aabb: box_,
            primitive,
            morton,
        }
    }
}

/// A branch in the BVH.
#[derive(Clone, Debug)]
pub struct Branch {
    /// The bounding box of this branch.
    pub box_: Aabb,
    /// The left child of this branch.
    pub left: Box<BvhNode>,
    /// The right child of this branch.
    pub right: Box<BvhNode>,
}

impl Branch {
    #[must_use]
    #[inline]
    /// Create a new branch.
    pub const fn new(box_: Aabb, left: Box<BvhNode>, right: Box<BvhNode>) -> Self {
        Self { box_, left, right }
    }
}

/// A node in the BVH.
#[derive(Clone, Debug)]
pub enum BvhNode {
    /// A leaf node at the end of a series of branches
    Leaf(Leaf),
    /// A branch node that connects to more nodes
    Branch(Branch),
}

impl BvhNode {
    #[must_use]
    #[inline]
    /// Creates a new branch for the BVH given two children.
    pub fn branch(right: Self, left: Self) -> Self {
        Self::Branch(Branch::new(
            *right.aabb() + *left.aabb(),
            Box::new(left),
            Box::new(right),
        ))
    }

    #[must_use]
    #[inline]
    /// Returns the bounding box of this node.
    pub const fn aabb(&self) -> &Aabb {
        match self {
            Self::Leaf(leaf) => &leaf.aabb,
            Self::Branch(branch) => &branch.box_,
        }
    }
}

/// A bounding volume hierarchy.
#[derive(Clone, Debug)]
pub struct Bvh {
    /// The number of leaves that the BVH has; used in tests
    pub num_leaves: usize,
    /// The root of the BVH.
    pub root: BvhNode,
}

#[inline]
fn global_aabb(boxes: &[Aabb]) -> Aabb {
    boxes.iter().copied().fold(boxes[0], |a, b| a + b)
}

impl Default for Bvh {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl Bvh {
    #[inline]
    pub const fn new() -> Self {
        Bvh {
            num_leaves: 0,
            root: BvhNode::Leaf(Leaf::new(
                Tri([Vec3A::ZERO; 3]),
                Aabb::new(Vec3A::ZERO, Vec3A::ZERO),
                0,
            )),
        }
    }

    #[inline]
    pub fn get_aabb(&self, trans: Transform) -> Aabb {
        let local_aabb = self.root.aabb();
        let local_half_extents = 0.5 * (local_aabb.max() - local_aabb.min());
        let local_center = 0.5 * (local_aabb.max() + local_aabb.min());

        let center = trans.transform_point3a(local_center);
        let extent = Vec3A::new(
            local_half_extents.dot(trans.matrix3.x_axis.abs()),
            local_half_extents.dot(trans.matrix3.y_axis.abs()),
            local_half_extents.dot(trans.matrix3.z_axis.abs()),
        );

        Aabb::new(center - extent, center + extent)
    }

    #[must_use]
    /// Creates a new BVH from a list of primitives.
    pub fn from(primitives: &[Tri]) -> Self {
        let num_leaves = primitives.len();

        let boxes: Vec<Aabb> = primitives.iter().copied().map(Into::into).collect();
        let global_box = global_aabb(&boxes);
        let morton = Morton::from(global_box);

        let mut sorted_leaves: Vec<Leaf> = primitives
            .iter()
            .copied()
            .zip(boxes)
            .map(|(primitive, box_)| Leaf::new(primitive, box_, morton.get_code(box_)))
            .collect();
        radsort::sort_by_key(&mut sorted_leaves, |leaf| leaf.morton);

        let root = Self::generate_hierarchy(&sorted_leaves, 0, num_leaves - 1);

        Self { num_leaves, root }
    }

    fn generate_hierarchy(sorted_leaves: &[Leaf], first: usize, last: usize) -> BvhNode {
        // If we're dealing with a single object, return the leaf node
        if first == last {
            return BvhNode::Leaf(sorted_leaves[first]);
        }

        // Determine where to split the range
        let split = first + ((last - first) / 2);

        // Process the resulting sub-ranges recursively
        let right = Self::generate_hierarchy(sorted_leaves, first, split);
        let left = Self::generate_hierarchy(sorted_leaves, split + 1, last);

        BvhNode::branch(right, left)
    }
}
