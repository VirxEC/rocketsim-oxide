use crate::bullet::collision::{
    dispatch::collision_object::CollisionObject,
    narrowphase::persistent_manifold::PersistentManifold,
};

pub trait CollisionAlgorithm {
    fn process_collision(
        self,
        body0: &CollisionObject,
        body1: &CollisionObject,
    ) -> Option<PersistentManifold>;
}
