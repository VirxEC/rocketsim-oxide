use super::collision_object::CollisionObject;
use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            collision_shape::CollisionShapes, triangle_callback::TriangleCallback,
            triangle_shape::TriangleShape,
        },
    },
    linear_math::aabb_util_2::{Aabb, test_aabb_against_aabb},
};

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: &'a CollisionObject,
    pub tri_obj: &'a CollisionObject,
    pub aabb: &'a Aabb,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: &'a CollisionObject,
        tri_obj: &'a CollisionObject,
        aabb: &'a Aabb,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj, is_swapped),
            convex_obj,
            tri_obj,
            aabb,
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
            let Some(CollisionShapes::Sphere(sphere_shape)) = self.convex_obj.get_collision_shape()
            else {
                unreachable!()
            };

            (
                self.convex_obj.get_world_transform().translation,
                sphere_shape.get_radius(),
            )
        };

        let Some(contact_info) =
            triangle.intersect_sphere(center, radius, self.manifold.contact_breaking_threshold)
        else {
            return true;
        };

        self.manifold.add_contact_point(
            self.convex_obj,
            self.tri_obj,
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
    convex_obj: &'a CollisionObject,
    concave_obj: &'a CollisionObject,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexConcaveCollisionAlgorithm<'a, T> {
    pub const fn new(
        convex_obj: &'a CollisionObject,
        concave_obj: &'a CollisionObject,
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
    fn process_collision(self) -> Option<PersistentManifold> {
        let Some(CollisionShapes::Sphere(sphere_shape)) = self.convex_obj.get_collision_shape()
        else {
            unreachable!()
        };

        let Some(CollisionShapes::TriangleMesh(tri_mesh)) = self.concave_obj.get_collision_shape()
        else {
            unreachable!()
        };

        let xform1 = self.convex_obj.get_world_transform();
        let xform2 = self.concave_obj.get_world_transform();
        let convex_in_triangle_space = xform1 * xform2;

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
            convex_triangle_callback
                .manifold
                .refresh_contact_points(self.convex_obj, self.concave_obj);
            Some(convex_triangle_callback.manifold)
        }
    }
}
