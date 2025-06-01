use super::collision_object::CollisionObject;
use crate::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        narrowphase::persistent_manifold::PersistentManifold,
        shapes::{collision_shape::CollisionShapes, triangle_callback::TriangleCallback},
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

    fn face_contains(points: &[Vec3A], n: Vec3A, obj_to_points: &[Vec3A; 3]) -> bool {
        let edges = [
            points[1] - points[0],
            points[2] - points[1],
            points[0] - points[2],
        ];

        let edge_normals = [edges[0].cross(n), edges[1].cross(n), edges[2].cross(n)];

        let r = [
            edge_normals[0].dot(obj_to_points[0]),
            edge_normals[1].dot(obj_to_points[1]),
            edge_normals[2].dot(obj_to_points[2]),
        ];

        (r[0] > 0. && r[1] > 0. && r[2] > 0.) || (r[0] <= 0. && r[1] <= 0. && r[2] <= 0.)
    }

    fn closest_point(points: &[Vec3A], obj_to_points: [Vec3A; 3], ab: Vec3A, ac: Vec3A) -> Vec3A {
        let d1 = ab.dot(obj_to_points[0]);
        let d2 = ac.dot(obj_to_points[0]);
        if d1 <= 0. && d2 <= 0. {
            return points[0];
        }

        let d3 = ab.dot(obj_to_points[1]);
        let d4 = ac.dot(obj_to_points[1]);
        if d3 >= 0. && d4 <= d3 {
            return points[1];
        }

        let d5 = ab.dot(obj_to_points[2]);
        let d6 = ac.dot(obj_to_points[2]);
        if d6 >= 0. && d5 <= d6 {
            return points[2];
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= 0. && d1 >= 0. && d3 <= 0. {
            let v = d1 / (d1 - d3);
            return points[0] + v * ab;
        }

        let vb = d5 * d2 - d1 * d6;
        if vb <= 0. && d2 >= 0. && d6 <= 0. {
            let v = d2 / (d2 - d6);
            return points[0] + v * ac;
        }

        let va = d3 * d6 - d5 * d4;
        if va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0. {
            let v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return points[1] + v * (points[2] - points[1]);
        }

        let denom = 1. / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        points[0] + v * ab + w * ac
    }
}

impl TriangleCallback for ConvexTriangleCallback {
    fn process_triangle(
        &mut self,
        triangle: &[glam::Vec3A],
        tri_aabb_min: Vec3A,
        tri_aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool {
        if !test_aabb_against_aabb(tri_aabb_min, tri_aabb_max, self.aabb_min, self.aabb_max) {
            return true;
        }

        let sphere_ref = self.manifold.body0.borrow();

        let sphere_col_shape = sphere_ref.get_collision_shape().unwrap().borrow();
        let CollisionShapes::Sphere(sphere_shape) = &*sphere_col_shape else {
            unreachable!()
        };

        let sphere_center = sphere_ref.get_world_transform().translation;
        let mut triangle_normal = (triangle[1] - triangle[0])
            .cross(triangle[2] - triangle[0])
            .normalize();
        let obj_to_center = sphere_center - triangle[0];
        let mut distance_from_plane = obj_to_center.dot(triangle_normal);

        if distance_from_plane < 0. {
            distance_from_plane *= -1.;
            triangle_normal *= -1.;
        }

        let radius = sphere_shape.get_radius();
        let radius_with_threshold = radius + self.manifold.contact_breaking_threshold;
        if distance_from_plane >= radius_with_threshold {
            return true;
        }

        let obj_to_points = [
            obj_to_center,
            sphere_center - triangle[1],
            sphere_center - triangle[2],
        ];

        let contact_point = if Self::face_contains(triangle, triangle_normal, &obj_to_points) {
            sphere_center - triangle_normal * distance_from_plane
        } else {
            let closest_point = Self::closest_point(
                triangle,
                obj_to_points,
                triangle[1] - triangle[0],
                triangle[2] - triangle[0],
            );
            let distance_sqr = (closest_point - sphere_center).length_squared();

            if distance_sqr < radius_with_threshold * radius_with_threshold {
                closest_point
            } else {
                return true;
            }
        };

        let contact_to_center = sphere_center - contact_point;
        let distance_sqr = contact_to_center.length_squared();

        if distance_sqr >= radius_with_threshold * radius_with_threshold {
            return true;
        }

        let contact_to_center = sphere_center - contact_point;
        let distance_sqr = contact_to_center.length_squared();

        if distance_sqr >= radius_with_threshold * radius_with_threshold {
            return true;
        }

        let (result_normal, depth) = if distance_sqr > f32::EPSILON {
            let distance = distance_sqr.sqrt();
            (contact_to_center / distance, -(radius - distance))
        } else {
            (triangle_normal, -radius)
        };

        drop(sphere_col_shape);
        drop(sphere_ref);

        self.manifold.add_contact_point(
            result_normal,
            contact_point,
            depth,
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
