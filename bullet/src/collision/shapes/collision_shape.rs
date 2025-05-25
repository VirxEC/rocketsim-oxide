use std::sync::Arc;

use super::{
    sphere_shape::SphereShape, static_plane_shape::StaticPlaneShape,
    triangle_mesh_shape::TriangleMeshShape,
};
use crate::collision::broadphase::broadphase_proxy::BroadphaseNativeTypes;
use glam::{Affine3A, Vec3A};

#[derive(Clone, Debug)]
pub struct CollisionShape {
    pub shape_type: BroadphaseNativeTypes,
    // pub user_pointer: *mut c_void,
    pub user_index: i32,
    pub user_index_2: i32,
    pub aabb_cached: bool,
    pub aabb_min_cache: Vec3A,
    pub aabb_max_cache: Vec3A,
    pub aabb_cache_trans: Affine3A,
}

impl Default for CollisionShape {
    fn default() -> Self {
        Self {
            shape_type: BroadphaseNativeTypes::InvalidShapeProxytype,
            user_index: -1,
            user_index_2: -1,
            aabb_cached: false,
            aabb_min_cache: Vec3A::ZERO,
            aabb_max_cache: Vec3A::ZERO,
            aabb_cache_trans: Affine3A::ZERO,
        }
    }
}

pub enum CollisionShapes {
    Sphere(SphereShape),
    StaticPlane(StaticPlaneShape),
    TriangleMesh(Arc<TriangleMeshShape>),
}

fn fast_compare_transforms(a: &Affine3A, b: &Affine3A) -> bool {
    a.translation == b.translation
        && a.matrix3.x_axis == b.matrix3.x_axis
        && a.matrix3.y_axis == b.matrix3.y_axis
}

impl CollisionShapes {
    pub fn reset_aabb_cache(&mut self) {
        match self {
            CollisionShapes::Sphere(mesh) => {
                mesh.convex_internal_shape
                    .convex_shape
                    .collision_shape
                    .aabb_cached = false
            }
            CollisionShapes::StaticPlane(_) | CollisionShapes::TriangleMesh(_) => {
                unreachable!();
            }
        }
    }

    pub fn get_collision_shape(&self) -> &CollisionShape {
        match self {
            CollisionShapes::Sphere(shape) => {
                &shape.convex_internal_shape.convex_shape.collision_shape
            }
            CollisionShapes::StaticPlane(shape) => &shape.concave_shape.collision_shape,
            CollisionShapes::TriangleMesh(shape) => &shape.concave_shape.collision_shape,
        }
    }

    pub fn get_collision_shape_mut(&mut self) -> &mut CollisionShape {
        match self {
            CollisionShapes::Sphere(shape) => {
                &mut shape.convex_internal_shape.convex_shape.collision_shape
            }
            CollisionShapes::StaticPlane(_) | CollisionShapes::TriangleMesh(_) => unreachable!(),
        }
    }

    pub fn get_aabb(&self, t: &Affine3A) -> (Vec3A, Vec3A) {
        if let CollisionShapes::Sphere(shape) = self {
            // If we're a sphere, its faster to just re-calculate
            return shape.get_aabb(t);
        }

        let cs = self.get_collision_shape();
        if !cs.aabb_cached || !fast_compare_transforms(t, &cs.aabb_cache_trans) {
            todo!("no aabb found for {t} in {cs:?}");
        } else {
            (cs.aabb_min_cache, cs.aabb_max_cache)
        }
    }

    pub fn get_shape_type(&self) -> BroadphaseNativeTypes {
        match self {
            CollisionShapes::Sphere(_) => BroadphaseNativeTypes::SphereShapeProxytype,
            CollisionShapes::StaticPlane(_) => BroadphaseNativeTypes::StaticPlaneProxytype,
            CollisionShapes::TriangleMesh(_) => BroadphaseNativeTypes::TriangleMeshShapeProxytype,
        }
    }
}
