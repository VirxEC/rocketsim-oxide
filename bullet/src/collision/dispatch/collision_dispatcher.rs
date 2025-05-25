use super::collision_configuration::CollisionConfiguration;
use crate::collision::broadphase::{
    broadphase_proxy::{BroadphaseNativeTypes, BroadphasePair},
    dispatcher::DispatcherInfo,
};

pub enum DispatcherFlags {
    CdStaticStaticReported = 1,
    CdUseRelativeContactBreakingThreshold = 2,
    CdDisableContactpoolDynamicAllocation = 4,
}

pub type NearCallback = fn(
    collision_pair: &mut BroadphasePair,
    dispatcher: &mut CollisionDispatcher,
    dispatch_info: &DispatcherInfo,
);

const MAX_BROADPHASE_COLLISION_TYPES: usize =
    BroadphaseNativeTypes::MaxBroadphaseCollisionTypes as usize;

pub struct CollisionDispatcher {
    dispatcher_flags: i32,
    // btAlignedObjectArray<btPersistentManifold*> m_manifoldsPtr;
    near_callback: NearCallback,
    // btPoolAllocator* m_collisionAlgorithmPoolAllocator;
    // btPoolAllocator* m_persistentManifoldPoolAllocator;
    // btCollisionAlgorithmCreateFunc* m_doubleDispatchContactPoints[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
    // btCollisionAlgorithmCreateFunc* m_doubleDispatchClosestPoints[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
    // btCollisionConfiguration* m_collisionConfiguration;
}

impl CollisionDispatcher {
    pub fn new(collision_configuration: &dyn CollisionConfiguration) -> Self {
        Self {
            dispatcher_flags: 0,
            near_callback: Self::default_near_callback,
        }
    }

    pub fn set_near_callback(&mut self, near_callback: NearCallback) {
        self.near_callback = near_callback;
    }

    fn default_near_callback(
        collision_pair: &mut BroadphasePair,
        dispatcher: &mut CollisionDispatcher,
        dispatch_info: &DispatcherInfo,
    ) {
        todo!()
    }
}
