use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{collision_shape::CollisionShapes, sphere_shape::SPHERE_RADIUS_MARGIN},
    },
    linear_math::AffineExt,
};
use glam::Vec3A;
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
        mut body0: &'a CollisionObject,
        mut body1: &'a CollisionObject,
    ) -> Option<PersistentManifold> {
        if self.is_swapped {
            mem::swap(&mut body0, &mut body1);
        }

        let Some(CollisionShapes::Sphere(sphere_ref)) = body0.get_collision_shape() else {
            unreachable!();
        };

        let Some(CollisionShapes::Compound(compound_ref)) = body1.get_collision_shape() else {
            unreachable!();
        };

        let world_to_box = self.obb_obj.world_transform.transpose();
        let sphere_from_local =
            world_to_box.transform_point3a(body0.get_world_transform().translation);
        let sphere_radius = sphere_ref.get_radius();

        let box_aabb = compound_ref.get_ident_aabb();

        let closest = sphere_from_local.clamp(box_aabb.min, box_aabb.max);
        let delta = sphere_from_local - closest;
        let dist_sq = delta.length_squared();

        let radius_with_threshold = sphere_radius + SPHERE_RADIUS_MARGIN;
        if dist_sq >= radius_with_threshold * radius_with_threshold {
            return None;
        }

        let dist = dist_sq.sqrt();
        let depth = sphere_radius - dist;
        let normal = if dist > f32::EPSILON {
            delta / dist
        } else {
            Vec3A::X
        };

        let mut manifold =
            PersistentManifold::new(self.sphere_obj, self.obb_obj.object, self.is_swapped);
        manifold.add_contact_point(normal, closest, depth, -1, -1, self.contact_added_callback);
        manifold.refresh_contact_points();

        Some(manifold)
    }
}
