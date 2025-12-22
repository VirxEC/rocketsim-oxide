use std::sync::Arc;

use glam::{Affine3A, Vec3A};

use super::{
    bvh_triangle_mesh_shape::BvhTriangleMeshShape, compound_shape::CompoundShape,
    sphere_shape::SphereShape, static_plane_shape::StaticPlaneShape,
};
use crate::bullet::{
    collision::{
        broadphase::BroadphaseNativeTypes,
        dispatch::ray_callbacks::{BridgeTriangleRaycastPacketCallback, RayResultCallback},
        shapes::sphere_shape::SPHERE_RADIUS_MARGIN,
    },
    linear_math::aabb_util_2::Aabb,
};

#[derive(Clone, Debug)]
pub struct CollisionShape {
    pub shape_type: BroadphaseNativeTypes,
    pub aabb_ident_cache: Option<Aabb>,
    pub aabb_cache: Option<Aabb>,
    pub aabb_cache_trans: Affine3A,
}

impl Default for CollisionShape {
    fn default() -> Self {
        Self {
            shape_type: BroadphaseNativeTypes::InvalidShapeProxytype,
            aabb_ident_cache: None,
            aabb_cache: None,
            aabb_cache_trans: Affine3A::ZERO,
        }
    }
}

pub enum CollisionShapes {
    Compound(Box<CompoundShape>),
    Sphere(SphereShape),
    StaticPlane(StaticPlaneShape),
    TriangleMesh(Arc<BvhTriangleMeshShape>),
}

fn fast_compare_transforms(a: &Affine3A, b: &Affine3A) -> bool {
    a.translation == b.translation
        && a.matrix3.x_axis == b.matrix3.x_axis
        && a.matrix3.y_axis == b.matrix3.y_axis
}

impl CollisionShapes {
    #[must_use]
    pub fn get_collision_shape(&self) -> &CollisionShape {
        match self {
            Self::Compound(shape) => &shape.collision_shape,
            Self::Sphere(shape) => &shape.convex_internal_shape.convex_shape.collision_shape,
            Self::StaticPlane(shape) => &shape.concave_shape.collision_shape,
            Self::TriangleMesh(shape) => shape.get_collision_shape(),
        }
    }

    #[must_use]
    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        match self {
            Self::Sphere(shape) => shape.get_aabb(t),
            Self::Compound(shape) => shape.get_aabb(t),
            _ => {
                let cs = self.get_collision_shape();
                debug_assert!(fast_compare_transforms(t, &cs.aabb_cache_trans));
                cs.aabb_cache.unwrap()
            }
        }
    }

    #[must_use]
    pub const fn get_shape_type(&self) -> BroadphaseNativeTypes {
        match self {
            Self::Compound(_) => BroadphaseNativeTypes::CompoundShapeProxytype,
            Self::Sphere(_) => BroadphaseNativeTypes::SphereShapeProxytype,
            Self::StaticPlane(_) => BroadphaseNativeTypes::StaticPlaneProxytype,
            Self::TriangleMesh(_) => BroadphaseNativeTypes::TriangleMeshShapeProxytype,
        }
    }

    fn get_bounding_sphere(&self) -> (Vec3A, f32) {
        match self {
            Self::Sphere(sphere) => (Vec3A::ZERO, sphere.get_radius() + SPHERE_RADIUS_MARGIN),
            Self::Compound(compound) => {
                let aabb = compound.get_ident_aabb();
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;

                (center, radius)
            }
            _ => {
                let aabb = self.get_collision_shape().aabb_ident_cache.unwrap();
                let center = (aabb.min + aabb.max) * 0.5;
                let radius = (aabb.max - aabb.min).length() * 0.5;

                (center, radius)
            }
        }
    }

    fn get_angular_motion_disc(&self) -> f32 {
        let (center, disc) = self.get_bounding_sphere();
        disc + center.length()
    }

    #[must_use]
    pub fn get_contact_breaking_threshold(&self, default_contact_threshold: f32) -> f32 {
        self.get_angular_motion_disc() * default_contact_threshold
    }

    #[must_use]
    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        match self {
            Self::Sphere(shape) => shape.local_get_supporting_vertex(vec),
            Self::Compound(shape) => shape
                .child
                .as_ref()
                .unwrap()
                .child_shape
                .local_get_supporting_vertex(vec),
            _ => todo!(),
        }
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_from_local: &[Vec3A; 4],
        ray_to_local: &[Vec3A; 4],
    ) {
        match self {
            Self::Compound(compound) => {
                compound.perform_raycast(result_callback, ray_from_local, ray_to_local);
            }
            Self::Sphere(sphere) => {
                sphere.perform_raycast(result_callback, ray_from_local, ray_to_local);
            }
            Self::StaticPlane(plane) => {
                plane.perform_raycast(result_callback, ray_from_local, ray_to_local);
            }
            Self::TriangleMesh(mesh) => {
                mesh.perform_raycast(result_callback, ray_from_local, ray_to_local);
            }
        }
    }
}
