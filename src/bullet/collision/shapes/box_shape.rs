use super::{
    collision_margin::CONVEX_DISTANCE_MARGIN, collision_shape::CollisionShape,
    convex_internal_shape::ConvexInternalShape, convex_shape::ConvexShape,
    polyhedral_convex_shape::PolyhedralConvexShape,
};
use crate::bullet::{
    collision::broadphase::broadphase_proxy::BroadphaseNativeTypes,
    linear_math::aabb_util_2::{Aabb, transform_aabb},
};
use glam::{Affine3A, Vec3A, Vec3Swizzles};

pub struct BoxShape {
    pub(crate) polyhedral_convex_shape: PolyhedralConvexShape,
}

impl BoxShape {
    pub fn new(box_half_extents: Vec3A) -> Self {
        Self {
            polyhedral_convex_shape: PolyhedralConvexShape {
                convex_internal_shape: ConvexInternalShape {
                    convex_shape: ConvexShape {
                        collision_shape: CollisionShape {
                            shape_type: BroadphaseNativeTypes::BoxShapeProxytype,
                            ..Default::default()
                        },
                    },
                    implicit_shape_dimensions: box_half_extents - CONVEX_DISTANCE_MARGIN,
                    collision_margin: {
                        let safe_margin = 0.1 * box_half_extents.min_element();
                        safe_margin.min(CONVEX_DISTANCE_MARGIN)
                    },
                },
            },
        }
    }

    #[inline]
    pub fn get_half_extents(&self) -> Vec3A {
        self.polyhedral_convex_shape
            .convex_internal_shape
            .implicit_shape_dimensions
            + self
                .polyhedral_convex_shape
                .convex_internal_shape
                .collision_margin
    }

    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        transform_aabb(
            self.polyhedral_convex_shape
                .convex_internal_shape
                .implicit_shape_dimensions,
            self.polyhedral_convex_shape
                .convex_internal_shape
                .collision_margin,
            t,
        )
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let l = 2.0 * self.get_half_extents();
        let yxx = l.yxx();
        let zzy = l.zzy();

        mass / 12.0 * (yxx * yxx + zzy * zzy)
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        let half_extents = self.get_half_extents();
        Vec3A::select(vec.cmpge(Vec3A::splat(0.0)), half_extents, -half_extents)
    }
}
