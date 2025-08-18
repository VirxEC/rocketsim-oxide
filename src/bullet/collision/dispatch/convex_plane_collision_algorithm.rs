use super::collision_object::CollisionObject;
use crate::bullet::collision::{
    broadphase::collision_algorithm::CollisionAlgorithm,
    dispatch::collision_object_wrapper::CollisionObjectWrapper,
    narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
    shapes::collision_shape::CollisionShapes,
};
use std::{cell::RefCell, rc::Rc};

pub struct ConvexPlaneCollisionAlgorithm<'a, T: ContactAddedCallback> {
    is_swapped: bool,
    convex_obj: CollisionObjectWrapper,
    plane_obj: Rc<RefCell<CollisionObject>>,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexPlaneCollisionAlgorithm<'a, T> {
    pub const fn new(
        convex_obj: CollisionObjectWrapper,
        plane_obj: Rc<RefCell<CollisionObject>>,
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
    fn process_collision(
        self,
        body0: &CollisionObject,
        body1: &CollisionObject,
    ) -> Option<PersistentManifold> {
        let (convex_obj, plane_obj) = if self.is_swapped {
            (body1, body0)
        } else {
            (body0, body1)
        };

        let mut manifold =
            PersistentManifold::new(self.convex_obj.object, self.plane_obj, self.is_swapped);

        let col_shape = convex_obj.get_collision_shape().unwrap();
        let Some(CollisionShapes::StaticPlane(plane_shape)) = plane_obj.get_collision_shape()
        else {
            unreachable!()
        };

        let plane_normal = plane_shape.get_plane_normal();
        let plane_constant = plane_shape.get_plane_constant();

        let plane_in_convex = self.convex_obj.world_transform.matrix3.transpose()
            * plane_obj.get_world_transform().matrix3;
        let convex_in_plane_trans =
            self.convex_obj.world_transform * plane_obj.get_world_transform();

        let vtx = col_shape.local_get_supporting_vertex(plane_in_convex * -plane_normal);
        let vtx_in_plane = convex_in_plane_trans.transform_point3a(vtx);
        let distance = plane_normal.dot(vtx_in_plane) - plane_constant;

        if distance < manifold.contact_breaking_threshold {
            let vtx_in_plane_projected = vtx_in_plane - distance * plane_normal;
            let vtx_in_plane_world = plane_obj
                .get_world_transform()
                .transform_point3a(vtx_in_plane_projected);
            let normal_on_surface_b = plane_obj.get_world_transform().matrix3 * plane_normal;

            manifold.add_contact_point(
                normal_on_surface_b,
                vtx_in_plane_world,
                distance,
                -1,
                -1,
                self.contact_added_callback,
            );
        }

        manifold.refresh_contact_points();

        if manifold.point_cache.is_empty() {
            None
        } else {
            Some(manifold)
        }
    }
}
