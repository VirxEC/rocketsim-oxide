use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{collision_shape::CollisionShapes, sphere_shape::SPHERE_RADIUS_MARGIN},
    },
    linear_math::{AffineExt, aabb_util_2::test_aabb_against_aabb},
};
use glam::{Affine3A, Vec3A};
use std::{cell::RefCell, mem, rc::Rc};

pub struct SphereObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    sphere_obj: Rc<RefCell<CollisionObject>>,
    obb_obj: CollisionObjectWrapper,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> SphereObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        sphere_obj: Rc<RefCell<CollisionObject>>,
        obb_obj: CollisionObjectWrapper,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            sphere_obj,
            obb_obj,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for SphereObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(
        self,
        mut sphere_obj: &'a CollisionObject,
        mut compound_obj: &'a CollisionObject,
    ) -> Option<PersistentManifold> {
        if self.is_swapped {
            mem::swap(&mut sphere_obj, &mut compound_obj);
        }

        let Some(CollisionShapes::Sphere(sphere_ref)) = sphere_obj.get_collision_shape() else {
            unreachable!();
        };

        let Some(CollisionShapes::Compound(compound_shape)) = compound_obj.get_collision_shape()
        else {
            unreachable!();
        };

        let sphere_trans = sphere_obj.get_world_transform();
        let aabb_1 = sphere_ref.get_aabb(sphere_trans);

        let org_trans = compound_obj.get_world_transform();
        let aabb_2 = compound_shape.get_aabb(org_trans);

        if !test_aabb_against_aabb(&aabb_1, &aabb_2) {
            return None;
        }

        let child = compound_shape.child.as_ref().unwrap();
        let child_trans = child.transform;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &child.child_shape;
        let box_aabb = box_shape.get_aabb(&Affine3A::IDENTITY);

        let sphere_from_local = new_child_world_trans.inv_x_form(sphere_trans.translation);

        let closest = sphere_from_local.clamp(box_aabb.min, box_aabb.max);
        let delta = sphere_from_local - closest;
        let dist_sq = delta.length_squared();

        let radius = sphere_ref.get_radius();
        let radius_with_threshold = radius + SPHERE_RADIUS_MARGIN;
        if dist_sq >= radius_with_threshold * radius_with_threshold {
            return None;
        }

        let dist = dist_sq.sqrt();
        let normal = if dist > f32::EPSILON {
            delta / dist
        } else {
            Vec3A::X
        };

        let normal_in_world = new_child_world_trans.transform_vector3a(normal);
        let point_in_world = new_child_world_trans.transform_point3a(closest);
        let depth = radius_with_threshold - dist;

        let mut manifold =
            PersistentManifold::new(self.sphere_obj, self.obb_obj.object, self.is_swapped);
        manifold.add_contact_point(
            normal_in_world,
            point_in_world,
            depth,
            -1,
            -1,
            self.contact_added_callback,
        );
        manifold.refresh_contact_points();

        Some(manifold)
    }
}
