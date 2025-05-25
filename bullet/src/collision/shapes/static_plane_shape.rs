use super::concave_shape::ConcaveShape;
use crate::collision::{
    broadphase::broadphase_proxy::BroadphaseNativeTypes, shapes::collision_shape::CollisionShape,
};
use glam::{Affine3A, Vec3A};

pub struct StaticPlaneShape {
    pub concave_shape: ConcaveShape,
    local_aabb_min: Vec3A,
    local_aabb_max: Vec3A,
    plane_normal: Vec3A,
    plane_constant: f32,
    is_single_axis: bool,
    single_axis_idx: usize,
    single_axis_backwards: bool,
}

impl StaticPlaneShape {
    pub fn new(plane_normal: Vec3A, plane_constant: f32) -> Self {
        debug_assert!(plane_normal.is_normalized());

        let [x, y, z]: [u32; 3] = plane_normal.abs().cmpge(Vec3A::splat(f32::EPSILON)).into();

        let (is_single_axis, single_axis_idx, single_axis_backwards) = if x + y + z == 1 {
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
            local_aabb_min: Vec3A::ZERO,
            local_aabb_max: Vec3A::ZERO,
        }
    }

    pub fn get_aabb(&self, t: &Affine3A) -> (Vec3A, Vec3A) {
        let mut aabb_min = Vec3A::MIN;
        let mut aabb_max = Vec3A::MAX;

        if self.is_single_axis {
            const PLANE_CONSTANT_OFFSET: f32 = 0.2;

            aabb_min[self.single_axis_idx] =
                t.translation[self.single_axis_idx] + self.plane_constant - PLANE_CONSTANT_OFFSET;
            aabb_max[self.single_axis_idx] =
                t.translation[self.single_axis_idx] + self.plane_constant + PLANE_CONSTANT_OFFSET;

            (if self.single_axis_backwards {
                aabb_max
            } else {
                aabb_min
            })[self.single_axis_idx] = if self.single_axis_backwards {
                f32::MAX
            } else {
                f32::MIN
            };
        }

        (aabb_min, aabb_max)
    }
}
