use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        dispatch::collision_object::CollisionObject,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::{AffineExt, aabb_util_2::Aabb},
};
use glam::Vec3A;
use std::mem;

pub struct SphereObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> SphereObbCollisionAlgorithm<'a, T> {
    pub const fn new(is_swapped: bool, contact_added_callback: &'a mut T) -> Self {
        Self {
            is_swapped,
            contact_added_callback,
        }
    }
}

fn sq_dist_point_aabb(p: Vec3A, b: Aabb) -> f32 {
    let min = b.min - p;
    let sq_dist_1 = Vec3A::select(p.cmplt(b.min), min * min, Vec3A::ZERO);

    let max = p - b.max;
    let sq_dist_2 = Vec3A::select(p.cmpgt(b.max), max * max, Vec3A::ZERO);

    sq_dist_1.element_sum() + sq_dist_2.element_sum()
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

        let world_to_box = body1.get_world_transform().transpose();
        let sphere_from_local =
            world_to_box.transform_point3a(body0.get_world_transform().translation);
        let sphere_radius = sphere_ref.get_radius();

        let box_aabb = compound_ref.get_ident_aabb();

        let dist_sq = sq_dist_point_aabb(sphere_from_local, box_aabb);
        if dist_sq > sphere_radius * sphere_radius {
            return None;
        }

        todo!("sphere/aabb collision")
    }
}
