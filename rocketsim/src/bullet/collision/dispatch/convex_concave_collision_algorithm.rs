use glam::{Affine3A, Vec3A};

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
    linear_math::{AffineExt, aabb_util_2::Aabb},
};

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: &'a CollisionObject,
    pub tri_obj: &'a CollisionObject,
    contact_added_callback: &'a mut T,
    sphere_center: Vec3A,
    sphere_radius: f32,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: &'a CollisionObject,
        tri_obj: &'a CollisionObject,
        sphere_center: Vec3A,
        sphere_radius: f32,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj, is_swapped),
            convex_obj,
            tri_obj,
            sphere_center,
            sphere_radius,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> TriangleCallback for ConvexTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        _tri_aabb: &Aabb,
        triangle_index: usize,
    ) {
        let Some(contact_info) = triangle.intersect_sphere(
            self.sphere_center,
            self.sphere_radius,
            self.manifold.contact_breaking_threshold,
        ) else {
            return;
        };

        let normal_on_b = self
            .tri_obj
            .get_world_transform()
            .transform_vector3a(contact_info.result_normal);
        let point_in_world = self
            .tri_obj
            .get_world_transform()
            .transform_point3a(contact_info.contact_point);

        self.manifold.add_contact_point(
            self.convex_obj,
            self.tri_obj,
            normal_on_b,
            point_in_world,
            contact_info.depth,
            -1,
            triangle_index as i32,
            self.contact_added_callback,
        );
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
        let xform2 = self.concave_obj.get_world_transform().transpose();
        let convex_in_triangle_space = Affine3A {
            matrix3: xform2.matrix3 * xform1.matrix3,
            translation: xform2.transform_point3a(xform1.translation),
        };

        let mut convex_triangle_callback = ConvexTriangleCallback::new(
            self.convex_obj,
            self.concave_obj,
            convex_in_triangle_space.translation,
            sphere_shape.get_radius(),
            self.is_swapped,
            self.contact_added_callback,
        );

        let aabb = sphere_shape.get_aabb(&convex_in_triangle_space);
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
