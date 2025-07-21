use super::{box_shape::BoxShape, collision_shape::CollisionShape};
use crate::bullet::collision::broadphase::broadphase_proxy::BroadphaseNativeTypes;
use glam::{Affine3A, Vec3A};

pub struct CompoundShapeChild {
    pub transform: Affine3A,
    pub child_shape: BoxShape,
    // child_shape_type: BroadphaseNativeTypes,
    pub child_margin: f32,
}

pub struct CompoundShape {
    pub collision_shape: CollisionShape,
    pub child: Option<CompoundShapeChild>,
    local_aabb_max: Vec3A,
    local_aabb_min: Vec3A,
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
            local_aabb_max: Vec3A::ZERO,
            local_aabb_min: Vec3A::ZERO,
            update_revision: 1,
            collision_margin: 0.0,
        }
    }

    pub fn add_child_shape(&mut self, local_transform: Affine3A, shape: BoxShape) {
        self.update_revision += 1;

        let (local_aabb_min, local_aabb_max) = shape.get_aabb(&local_transform);

        self.local_aabb_min = self.local_aabb_min.min(local_aabb_min);
        self.local_aabb_max = self.local_aabb_max.min(local_aabb_max);

        self.child = Some(CompoundShapeChild {
            transform: local_transform,
            child_margin: shape
                .polyhedral_convex_shape
                .convex_internal_shape
                .collision_margin,
            child_shape: shape,
        });
    }

    pub fn get_aabb(&self, trans: &Affine3A) -> (Vec3A, Vec3A) {
        let local_half_extents =
            0.5 * (self.local_aabb_max - self.local_aabb_min) + Vec3A::splat(self.collision_margin);
        let local_center = 0.5 * (self.local_aabb_max + self.local_aabb_min);

        let abs_b = trans.matrix3.abs();
        let center = trans.transform_point3a(local_center);
        let extent = abs_b * local_half_extents;

        (center - extent, center + extent)
    }
}
