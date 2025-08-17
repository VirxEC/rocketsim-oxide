use super::{box_shape::BoxShape, collision_shape::CollisionShape};
use crate::bullet::{
    collision::broadphase::broadphase_proxy::BroadphaseNativeTypes, linear_math::aabb_util_2::Aabb,
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
}
