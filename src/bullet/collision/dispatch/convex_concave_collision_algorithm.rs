use super::collision_object::CollisionObject;
use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        narrowphase::persistent_manifold::PersistentManifold,
        shapes::{
            collision_shape::CollisionShapes, triangle_callback::TriangleCallback,
            triangle_shape::TriangleShape,
        },
    },
    linear_math::{AffineTranspose, aabb_util_2::test_aabb_against_aabb},
};
use glam::{Affine3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

struct ConvexTriangleCallback {
    manifold: PersistentManifold,
    aabb_min: Vec3A,
    aabb_max: Vec3A,
    is_swapped: bool,
}

impl ConvexTriangleCallback {
    fn new(
        convex_obj: Rc<RefCell<CollisionObject>>,
        tri_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj, is_swapped),
            is_swapped,
            aabb_max: Vec3A::ZERO,
            aabb_min: Vec3A::ZERO,
        }
    }
}

impl TriangleCallback for ConvexTriangleCallback {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb_min: Vec3A,
        tri_aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool {
        if !test_aabb_against_aabb(tri_aabb_min, tri_aabb_max, self.aabb_min, self.aabb_max) {
            return true;
        }

        let (center, radius) = {
            let sphere_ref = self.manifold.body0.borrow();
            let sphere_col_shape = sphere_ref.get_collision_shape().unwrap().borrow();
            let CollisionShapes::Sphere(sphere_shape) = &*sphere_col_shape else {
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
            part_id as i32,
            -1,
            triangle_index as i32,
        );

        true
    }
}

pub struct ConvexConcaveCollisionAlgorithm {
    convex_triangle_callback: ConvexTriangleCallback,
}

impl ConvexConcaveCollisionAlgorithm {
    pub fn new(
        convex_obj: Rc<RefCell<CollisionObject>>,
        concave_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
    ) -> Self {
        Self {
            convex_triangle_callback: ConvexTriangleCallback::new(
                convex_obj,
                concave_obj,
                is_swapped,
            ),
        }
    }
}

impl CollisionAlgorithm for ConvexConcaveCollisionAlgorithm {
    fn into_manifold(self) -> PersistentManifold {
        self.convex_triangle_callback.manifold
    }

    fn process_collision(&mut self, body0: &CollisionObject, body1: &CollisionObject) {
        let (sphere_obj, tris_obj) = if self.convex_triangle_callback.is_swapped {
            (body1, body0)
        } else {
            (body0, body1)
        };

        let sphere_col_shape = sphere_obj.get_collision_shape().unwrap().borrow();
        let CollisionShapes::Sphere(sphere_shape) = &*sphere_col_shape else {
            unreachable!()
        };

        let tri_col_shape = tris_obj.get_collision_shape().unwrap().borrow();
        let CollisionShapes::TriangleMesh(tri_mesh) = &*tri_col_shape else {
            unreachable!()
        };

        let xform1 = tris_obj.get_world_transform().transpose();
        let xform2 = sphere_obj.get_world_transform();
        let convex_in_triangle_space = Affine3A {
            matrix3: xform1.matrix3 * xform2.matrix3,
            translation: xform1.transform_point3a(xform2.translation),
        };

        let (aabb_min, aabb_max) = sphere_shape.get_aabb(&convex_in_triangle_space);

        self.convex_triangle_callback.aabb_min = aabb_min;
        self.convex_triangle_callback.aabb_max = aabb_max;

        tri_mesh.process_all_triangles(&mut self.convex_triangle_callback, aabb_min, aabb_max);

        self.convex_triangle_callback
            .manifold
            .refresh_contact_points();
    }
}
