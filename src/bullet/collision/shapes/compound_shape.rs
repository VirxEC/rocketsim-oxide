use std::{f32, mem};

use super::{box_shape::BoxShape, collision_shape::CollisionShape};
use crate::bullet::{
    collision::{
        broadphase::broadphase_proxy::BroadphaseNativeTypes,
        dispatch::collision_world::{BridgeTriangleRaycastCallback, RayResultCallback},
    },
    linear_math::aabb_util_2::{Aabb, test_aabb_against_aabb},
};
use glam::{Affine3A, Vec3A};

pub struct CompoundShapeChild {
    pub transform: Affine3A,
    pub child_shape: BoxShape,
    // child_shape_type: BroadphaseNativeTypes,
    // pub child_margin: f32,
}

pub struct CompoundShape {
    pub collision_shape: CollisionShape,
    pub child: Option<CompoundShapeChild>,
    local_aabb: Aabb,
    update_revision: u32,
    collision_margin: f32,
}

impl CompoundShape {
    pub fn new() -> Self {
        Self {
            collision_shape: CollisionShape {
                shape_type: BroadphaseNativeTypes::CompoundShapeProxytype,
                ..Default::default()
            },
            child: None,
            local_aabb: Aabb::ZERO,
            update_revision: 1,
            collision_margin: 0.0,
        }
    }

    pub fn add_child_shape(&mut self, local_transform: Affine3A, shape: BoxShape) {
        self.update_revision += 1;

        let local_aabb = shape.get_aabb(&local_transform);
        self.local_aabb += local_aabb;

        self.child = Some(CompoundShapeChild {
            transform: local_transform,
            // child_margin: shape
            //     .polyhedral_convex_shape
            //     .convex_internal_shape
            //     .collision_margin,
            child_shape: shape,
        });
    }

    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        let local_half_extents =
            0.5 * (self.local_aabb.max - self.local_aabb.min) + Vec3A::splat(self.collision_margin);
        let local_center = 0.5 * (self.local_aabb.max + self.local_aabb.min);

        let abs_b = trans.matrix3.abs();
        let center = trans.transform_point3a(local_center);
        let extent = abs_b * local_half_extents;

        Aabb {
            min: center - extent,
            max: center + extent,
        }
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastCallback<T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
    ) {
        let ray_aabb = Aabb::new(ray_source.min(ray_target), ray_source.max(ray_target));
        if !test_aabb_against_aabb(&ray_aabb, &self.local_aabb) {
            return;
        }

        let delta = ray_target - ray_source;
        let dist = delta.length();
        let dir = delta / dist;

        // implementation of the slab method to handle `dir` potentially having elements that are `0`
        let mut tenter = 0f32;
        let mut texit = dist;
        let mut hit_axis = 0usize;

        for axis in 0..3 {
            let origin = ray_source[axis];
            let dir_i = dir[axis];
            let min = self.local_aabb.min[axis];
            let max = self.local_aabb.max[axis];

            if dir_i.abs() < 1e-8 {
                // parallel - if the origin not within slab, no hit
                if origin < min || origin > max {
                    return;
                }

                // Axis does not clip the interval
                continue;
            }

            let inv = 1.0 / dir_i;
            let mut t1 = (min - origin) * inv;
            let mut t2 = (max - origin) * inv;
            if t1 > t2 {
                mem::swap(&mut t1, &mut t2);
            }

            texit = texit.min(t2);
            if t1 > tenter {
                tenter = t1;
                hit_axis = axis;
            }

            if tenter > texit || texit < 0.0 {
                return;
            }
        }

        tenter = tenter.max(0.0);
        if tenter > dist {
            return;
        }

        let mut hit_normal = Vec3A::ZERO;
        hit_normal[hit_axis] = -dir[hit_axis].signum();

        let hit_fraction = tenter / dist;
        result_callback.report_hit(hit_normal, hit_fraction);
    }
}
