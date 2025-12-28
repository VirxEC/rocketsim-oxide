use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        dispatch::collision_object::CollisionObject,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            compound_shape::CompoundShape,
            sphere_shape::{SPHERE_RADIUS_MARGIN, SphereShape},
        },
    },
    linear_math::AffineExt,
};

pub struct SphereObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    sphere_obj: &'a CollisionObject,
    sphere_shape: &'a SphereShape,
    obb_obj: &'a CollisionObject,
    obb_shape: &'a CompoundShape,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> SphereObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        sphere_obj: &'a CollisionObject,
        sphere_shape: &'a SphereShape,
        obb_obj: &'a CollisionObject,
        obb_shape: &'a CompoundShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            sphere_obj,
            sphere_shape,
            obb_obj,
            obb_shape,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for SphereObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(self) -> Option<PersistentManifold> {
        let sphere_trans = self.sphere_obj.get_world_transform();
        let aabb_1 = self.sphere_shape.get_aabb(sphere_trans);

        let org_trans = self.obb_obj.get_world_transform();
        let aabb_2 = self.obb_shape.get_aabb(org_trans);

        if !aabb_1.intersects(&aabb_2) {
            return None;
        }

        let child_trans = &self.obb_shape.child_transform;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &self.obb_shape.child_shape;
        let box_extents = box_shape.get_half_extents_no_margin();

        let sphere_from_local = new_child_world_trans.inv_x_form(sphere_trans.translation);

        let closest = sphere_from_local.clamp(-box_extents, box_extents);
        let delta = sphere_from_local - closest;
        let dist_sq = delta.length_squared();

        let radius = self.sphere_shape.get_radius();
        let radius_with_threshold = radius + box_shape.get_margin();
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
        let depth = -(radius_with_threshold - dist);

        let mut manifold = PersistentManifold::new(self.sphere_obj, self.obb_obj, self.is_swapped);
        manifold.add_contact_point(
            self.sphere_obj,
            self.obb_obj,
            normal_in_world,
            point_in_world,
            depth,
            -1,
            -1,
            self.contact_added_callback,
        );
        manifold.refresh_contact_points(self.sphere_obj, self.obb_obj);

        if manifold.point_cache.is_empty() {
            None
        } else {
            Some(manifold)
        }
    }
}
