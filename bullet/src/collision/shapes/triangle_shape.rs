use super::{
    collision_shape::CollisionShape, convex_internal_shape::ConvexInternalShape,
    convex_polyhedron::ConvexPolyhedron, convex_shape::ConvexShape,
    polyhedral_convex_shape::PolyhedralConvexShape,
};
use crate::collision::broadphase::broadphase_proxy::BroadphaseNativeTypes;
use glam::Vec3A;

pub struct TriangleShape {
    polyhedral_convex_shape: PolyhedralConvexShape,
    vertices1: [Vec3A; 3],
}

impl TriangleShape {
    pub fn new(vertices: [Vec3A; 3]) -> Self {
        Self {
            polyhedral_convex_shape: PolyhedralConvexShape {
                convex_internal_shape: ConvexInternalShape {
                    convex_shape: ConvexShape {
                        collision_shape: CollisionShape {
                            shape_type: BroadphaseNativeTypes::TriangleShapeProxytype,
                            ..Default::default()
                        },
                    },
                    ..Default::default()
                },
                polyhedron: ConvexPolyhedron::default(),
            },
            vertices1: vertices,
        }
    }

    pub fn calc_normal(&self) -> Vec3A {
        let normal =
            (self.vertices1[1] - self.vertices1[0]).cross(self.vertices1[2] - self.vertices1[0]);
        normal.normalize()
    }
}
