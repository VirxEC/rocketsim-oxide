use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        dispatch::collision_object::CollisionObject,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::{AffineExt, aabb_util_2::test_aabb_against_aabb},
};

pub struct AabbObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> AabbObbCollisionAlgorithm<'a, T> {
    pub const fn new(contact_added_callback: &'a mut T) -> Self {
        Self {
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for AabbObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(
        self,
        body0: &'a CollisionObject,
        body1: &'a CollisionObject,
    ) -> Option<PersistentManifold> {
        let Some(CollisionShapes::Compound(compound_0_ref)) = body0.get_collision_shape() else {
            unreachable!();
        };

        let Some(CollisionShapes::Compound(compound_1_ref)) = body1.get_collision_shape() else {
            unreachable!();
        };

        let world_to_box_0 = body0.get_world_transform().transpose();
        let box_1_from_local = world_to_box_0 * body1.get_world_transform();

        let box_0 = compound_0_ref.get_ident_aabb();
        let box_1_aabb = compound_1_ref.get_aabb(&box_1_from_local);
        if !test_aabb_against_aabb(&box_0, &box_1_aabb) {
            return None;
        }

        todo!("aabb/obb collision")
    }
}
