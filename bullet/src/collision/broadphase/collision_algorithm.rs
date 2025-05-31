use crate::collision::{
    dispatch::collision_object::CollisionObject,
    narrowphase::persistent_manifold::PersistentManifold,
};

pub trait CollisionAlgorithm {
    fn into_manifold(self) -> PersistentManifold;
    fn process_collision(&mut self, body0: &CollisionObject, body1: &CollisionObject);
}
