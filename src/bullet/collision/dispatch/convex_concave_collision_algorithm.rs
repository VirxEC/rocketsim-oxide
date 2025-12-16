use std::{cell::RefCell, rc::Rc};

use glam::Affine3A;

use super::collision_object::CollisionObject;
use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            collision_shape::CollisionShapes, triangle_callback::TriangleCallback,
            triangle_shape::TriangleShape,
        },
    },
    linear_math::{
        AffineExt,
        aabb_util_2::{Aabb, test_aabb_against_aabb},
    },
};

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub aabb: &'a Aabb,
    // is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: Rc<RefCell<CollisionObject>>,
        tri_obj: Rc<RefCell<CollisionObject>>,
        aabb: &'a Aabb,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj, is_swapped),
            aabb,
            // is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> TriangleCallback for ConvexTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb: &Aabb,
        triangle_index: usize,
    ) -> bool {
        if !test_aabb_against_aabb(tri_aabb, self.aabb) {
            return true;
        }

        let (center, radius) = {
            let sphere_ref = self.manifold.body0.borrow();
            let Some(CollisionShapes::Sphere(sphere_shape)) = sphere_ref.get_collision_shape()
            else {
                unreachable!()
            };

            (
                sphere_ref.get_world_transform().translation,
                sphere_shape.get_radius(),
            )
        };

        let Some(contact_info) =
            triangle.intersect_sphere(center, radius, self.manifold.contact_breaking_threshold)
        else {
            return true;
        };

        self.manifold.add_contact_point(
            contact_info.result_normal,
            contact_info.contact_point,
            contact_info.depth,
            -1,
            triangle_index as i32,
            self.contact_added_callback,
        );

        true
    }
}

pub struct ConvexConcaveCollisionAlgorithm<'a, T: ContactAddedCallback> {
    convex_obj: Rc<RefCell<CollisionObject>>,
    concave_obj: Rc<RefCell<CollisionObject>>,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexConcaveCollisionAlgorithm<'a, T> {
    pub const fn new(
        convex_obj: Rc<RefCell<CollisionObject>>,
        concave_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            convex_obj,
            concave_obj,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for ConvexConcaveCollisionAlgorithm<'_, T> {
    fn process_collision(
        self,
        body0: &CollisionObject,
        body1: &CollisionObject,
    ) -> Option<PersistentManifold> {
        let (sphere_obj, tris_obj) = if self.is_swapped {
            (body1, body0)
        } else {
            (body0, body1)
        };

        let Some(CollisionShapes::Sphere(sphere_shape)) = sphere_obj.get_collision_shape() else {
            unreachable!()
        };

        let Some(CollisionShapes::TriangleMesh(tri_mesh)) = tris_obj.get_collision_shape() else {
            unreachable!()
        };

        let xform1 = tris_obj.get_world_transform().transpose();
        let xform2 = sphere_obj.get_world_transform();
        let convex_in_triangle_space = Affine3A {
            matrix3: xform1.matrix3 * xform2.matrix3,
            translation: xform1.transform_point3a(xform2.translation),
        };

        let aabb = sphere_shape.get_aabb(&convex_in_triangle_space);
        let mut convex_triangle_callback = ConvexTriangleCallback::new(
            self.convex_obj,
            self.concave_obj,
            &aabb,
            self.is_swapped,
            self.contact_added_callback,
        );

        tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);

        if convex_triangle_callback.manifold.point_cache.is_empty() {
            None
        } else {
            convex_triangle_callback.manifold.refresh_contact_points();
            Some(convex_triangle_callback.manifold)
        }
    }
}
