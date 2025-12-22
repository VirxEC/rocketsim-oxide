use std::f32;

use glam::{Affine3A, Vec3A};

use super::{box_shape::BoxShape, collision_shape::CollisionShape};
use crate::bullet::{
    collision::{
        broadphase::BroadphaseNativeTypes,
        dispatch::ray_callbacks::{BridgeTriangleRaycastPacketCallback, RayResultCallback},
    },
    linear_math::{
        aabb_util_2::{Aabb, intersect_ray_aabb_packet, test_aabb_against_aabb},
        ray_packet::RayInfo,
    },
};

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

    pub fn get_ident_aabb(&self) -> Aabb {
        let margin = Vec3A::splat(self.collision_margin);
        Aabb {
            min: self.local_aabb.min - margin,
            max: self.local_aabb.max + margin,
        }
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_info: &RayInfo,
    ) {
        let box_aabb = self.get_ident_aabb();
        if !test_aabb_against_aabb(&ray_info.aabb, &box_aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mask =
            intersect_ray_aabb_packet(&origins, &inv_dirs, &box_aabb, result_callback.hit_fraction);

        for i in 0..4 {
            if (mask & (1 << i)) == 0 {
                continue;
            }

            self.internal_perform_raycast(
                result_callback,
                &box_aabb,
                ray_info.ray_sources[i],
                ray_info.ray_targets[i],
                i,
            );
        }
    }

    fn internal_perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        box_aabb: &Aabb,
        ray_source: Vec3A,
        ray_target: Vec3A,
        ray_idx: usize,
    ) {
        let delta = ray_target - ray_source;
        let dist = delta.length();
        let dir = delta / dist;

        // implementation of the slab method to handle `dir` potentially having elements that are `0`
        let mut tenter = 0f32;
        let mut texit = dist;
        let mut hit_axis = 0usize;

        let inv = 1.0 / dir;
        let t1 = (box_aabb.min - ray_source) * inv;
        let t2 = (box_aabb.max - ray_source) * inv;

        let tmin = t1.min(t2);
        let tmax = t1.max(t2);

        let is_neg = tmax.is_negative_bitmask();
        let is_finite: [bool; 3] = inv.is_finite_mask().into();
        for axis in 0..3 {
            if !is_finite[axis] {
                let origin = ray_source[axis];
                let min = box_aabb.min[axis];
                let max = box_aabb.max[axis];

                // parallel - if the origin not within slab, no hit
                if min > origin || origin > max {
                    return;
                }

                // Axis does not clip the interval
                continue;
            }

            if is_neg & (1 << axis) != 0 {
                return;
            }

            texit = texit.min(tmax[axis]);
            if tmin[axis] > tenter {
                tenter = tmin[axis];
                hit_axis = axis;
            }

            if tenter > texit {
                return;
            }
        }

        if tenter > dist {
            return;
        }

        let mut hit_normal = Vec3A::ZERO;
        hit_normal[hit_axis] = -dir[hit_axis].signum();

        let hit_fraction = tenter.max(0.0) / dist;
        result_callback.report_hit(hit_normal, hit_fraction, ray_idx);
    }
}
