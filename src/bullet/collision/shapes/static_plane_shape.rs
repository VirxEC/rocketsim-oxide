use super::concave_shape::ConcaveShape;
use crate::bullet::{
    collision::{
        broadphase::broadphase_proxy::BroadphaseNativeTypes,
        dispatch::collision_world::{BridgeTriangleRaycastCallback, RayResultCallback},
        shapes::collision_shape::CollisionShape,
    },
    linear_math::{LARGE_FLOAT, aabb_util_2::test_aabb_against_aabb},
};
use glam::{Affine3A, Vec3A};

pub struct StaticPlaneShape {
    pub concave_shape: ConcaveShape,
    // local_aabb_min: Vec3A,
    // local_aabb_max: Vec3A,
    plane_normal: Vec3A,
    plane_constant: f32,
    is_single_axis: bool,
    single_axis_idx: usize,
    single_axis_backwards: bool,
}

impl StaticPlaneShape {
    #[must_use]
    pub fn new(plane_normal: Vec3A, plane_constant: f32) -> Self {
        debug_assert!(plane_normal.is_normalized());

        let [x, y, z]: [bool; 3] = plane_normal.abs().cmpge(Vec3A::splat(f32::EPSILON)).into();

        let (is_single_axis, single_axis_idx, single_axis_backwards) =
            if u8::from(x) + u8::from(y) + u8::from(z) == 1 {
                let axis = plane_normal.abs().max_position();

                (true, axis, plane_normal[axis].is_sign_negative())
            } else {
                (false, 0, false)
            };

        Self {
            concave_shape: ConcaveShape {
                collision_shape: CollisionShape {
                    shape_type: BroadphaseNativeTypes::StaticPlaneProxytype,
                    ..Default::default()
                },
                ..Default::default()
            },
            plane_normal,
            plane_constant,
            is_single_axis,
            single_axis_idx,
            single_axis_backwards,
            // local_aabb_min: Vec3A::ZERO,
            // local_aabb_max: Vec3A::ZERO,
        }
    }

    #[must_use]
    pub fn get_aabb(&self, t: &Affine3A) -> (Vec3A, Vec3A) {
        let mut aabb_min = Vec3A::splat(-LARGE_FLOAT);
        let mut aabb_max = Vec3A::splat(LARGE_FLOAT);

        if self.is_single_axis {
            const PLANE_CONSTANT_OFFSET: f32 = 0.2;

            aabb_min[self.single_axis_idx] =
                t.translation[self.single_axis_idx] + self.plane_constant - PLANE_CONSTANT_OFFSET;
            aabb_max[self.single_axis_idx] =
                t.translation[self.single_axis_idx] + self.plane_constant + PLANE_CONSTANT_OFFSET;

            (if self.single_axis_backwards {
                &mut aabb_max
            } else {
                &mut aabb_min
            })[self.single_axis_idx] = if self.single_axis_backwards {
                LARGE_FLOAT
            } else {
                -LARGE_FLOAT
            };
        }

        (aabb_min, aabb_max)
    }

    #[must_use]
    pub const fn get_plane_normal(&self) -> Vec3A {
        self.plane_normal
    }

    #[must_use]
    pub const fn get_plane_constant(&self) -> f32 {
        self.plane_constant
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastCallback<T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
    ) {
        let aabb_min = self.concave_shape.collision_shape.aabb_min_cache;
        let aabb_max = self.concave_shape.collision_shape.aabb_max_cache;

        let ray_aabb_min = ray_source.min(ray_target);
        let ray_aabb_max = ray_source.max(ray_target);

        if !test_aabb_against_aabb(ray_aabb_min, ray_aabb_max, aabb_min, aabb_max) {
            return;
        }

        let delta = ray_target - ray_source;
        let dist = delta.length();
        let ray_direction = delta / dist;

        let dir_align = self.plane_normal.dot(ray_direction);
        if dir_align.abs() < f32::EPSILON {
            return;
        }

        let normal_start = self.plane_normal.dot(ray_source);
        let t = -normal_start / dir_align;
        if t < 0.0 {
            return;
        }

        let hit_fraction = t / dist;
        result_callback.report_hit(self.plane_normal, hit_fraction);
    }
}
