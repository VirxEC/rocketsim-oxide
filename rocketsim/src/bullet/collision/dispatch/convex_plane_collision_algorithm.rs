use glam::Affine3A;

use super::collision_object::CollisionObject;
use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        dispatch::collision_object_wrapper::CollisionObjectWrapper,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::aabb_util_2::test_aabb_against_aabb,
};

pub struct ConvexPlaneCollisionAlgorithm<'a, T: ContactAddedCallback> {
    is_swapped: bool,
    convex_obj: CollisionObjectWrapper<'a>,
    plane_obj: &'a CollisionObject,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexPlaneCollisionAlgorithm<'a, T> {
    pub const fn new(
        convex_obj: CollisionObjectWrapper<'a>,
        plane_obj: &'a CollisionObject,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            is_swapped,
            convex_obj,
            plane_obj,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for ConvexPlaneCollisionAlgorithm<'_, T> {
    fn process_collision(self) -> Option<PersistentManifold> {
        let col_shape = self.convex_obj.object.get_collision_shape();
        let CollisionShapes::StaticPlane(plane_shape) = self.plane_obj.get_collision_shape() else {
            unreachable!()
        };

        let convex_aabb = col_shape.get_aabb(&self.convex_obj.world_transform);
        let plane_aabb = plane_shape
            .concave_shape
            .collision_shape
            .aabb_cache
            .unwrap();
        if !test_aabb_against_aabb(&convex_aabb, &plane_aabb) {
            return None;
        }

        let plane_normal = plane_shape.get_plane_normal();
        let plane_constant = plane_shape.get_plane_constant();

        let plane_trans = self.plane_obj.get_world_transform();
        let plane_in_convex =
            self.convex_obj.world_transform.matrix3.transpose() * plane_trans.matrix3;
        let convex_in_plane_trans = Affine3A {
            matrix3: plane_trans.matrix3.transpose() * self.convex_obj.world_transform.matrix3,
            translation: plane_trans.matrix3 * self.convex_obj.world_transform.translation
                - plane_trans.translation,
        };

        let vtx = col_shape.local_get_supporting_vertex(plane_in_convex * -plane_normal);
        let vtx_in_plane = convex_in_plane_trans.transform_point3a(vtx);
        let distance = plane_normal.dot(vtx_in_plane) - plane_constant;

        let mut manifold =
            PersistentManifold::new(self.convex_obj.object, self.plane_obj, self.is_swapped);
        if distance >= manifold.contact_breaking_threshold {
            return None;
        }

        let vtx_in_plane_projected = vtx_in_plane - distance * plane_normal;
        let vtx_in_plane_world = self
            .plane_obj
            .get_world_transform()
            .transform_point3a(vtx_in_plane_projected);
        let normal_on_surface_b = self.plane_obj.get_world_transform().matrix3 * plane_normal;

        manifold.add_contact_point(
            self.convex_obj.object,
            self.plane_obj,
            normal_on_surface_b,
            vtx_in_plane_world,
            distance,
            -1,
            -1,
            self.contact_added_callback,
        );

        manifold.refresh_contact_points(self.convex_obj.object, self.plane_obj);
        Some(manifold)
    }
}
