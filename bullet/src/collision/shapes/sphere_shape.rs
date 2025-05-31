use super::{
    collision_shape::CollisionShape, convex_internal_shape::ConvexInternalShape,
    convex_shape::ConvexShape,
};
use crate::collision::broadphase::broadphase_proxy::BroadphaseNativeTypes;
use glam::{Affine3A, Vec3A};

pub struct SphereShape {
    pub convex_internal_shape: ConvexInternalShape,
}

impl SphereShape {
    pub fn new(radius: f32) -> Self {
        Self {
            convex_internal_shape: ConvexInternalShape {
                convex_shape: ConvexShape {
                    collision_shape: CollisionShape {
                        shape_type: BroadphaseNativeTypes::SphereShapeProxytype,
                        ..Default::default()
                    },
                },
                implicit_shape_dimensions: Vec3A::new(radius, 0.0, 0.0),
                collision_margin: radius,
                ..Default::default()
            },
        }
    }

    pub fn get_radius(&self) -> f32 {
        self.convex_internal_shape.implicit_shape_dimensions.x
    }

    pub fn get_margin(&self) -> f32 {
        self.get_radius()
    }

    pub fn get_aabb(&self, t: &Affine3A) -> (Vec3A, Vec3A) {
        let center = t.translation;
        let margin = self.get_margin() + 0.08;
        let extent = Vec3A::splat(margin);

        (center - extent, center + extent)
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vec3A {
        Vec3A::splat(0.4 * mass * self.get_margin() * self.get_margin())
    }

    pub fn local_get_support_vertex(&self, vec: Vec3A) -> Vec3A {
        self.get_margin() * vec.try_normalize().unwrap()
    }
}
