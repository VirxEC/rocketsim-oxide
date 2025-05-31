use std::{cell::RefCell, rc::Rc};

use super::{
    collision_object::CollisionObject,
    convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
};
use crate::collision::{
    broadphase::{
        broadphase_proxy::{BroadphaseNativeTypes, BroadphasePair},
        collision_algorithm::CollisionAlgorithm,
        dispatcher::{DispatchFunc, DispatcherInfo},
        overlapping_pair_cache::{OverlapCallback, OverlappingPairCache},
    },
    narrowphase::persistent_manifold::PersistentManifold,
};

pub enum DispatcherFlags {
    CdStaticStaticReported = 1,
    CdUseRelativeContactBreakingThreshold = 2,
    CdDisableContactpoolDynamicAllocation = 4,
}

pub type NearCallback = fn(
    collision_pair: &BroadphasePair,
    dispatcher: &mut CollisionDispatcher,
    dispatch_info: &DispatcherInfo,
);

const MAX_BROADPHASE_COLLISION_TYPES: usize =
    BroadphaseNativeTypes::MaxBroadphaseCollisionTypes as usize;

struct CollisionPairCallback<'a> {
    dispatch_info: &'a DispatcherInfo,
    dispatcher: &'a mut CollisionDispatcher,
}

impl OverlapCallback for CollisionPairCallback<'_> {
    fn process_overlap(&mut self, pair: &BroadphasePair) -> bool {
        self.dispatcher.near_callback(pair, self.dispatch_info);
        false
    }
}

#[derive(Default)]
pub struct CollisionDispatcher {
    dispatcher_flags: i32,
    // btAlignedObjectArray<btPersistentManifold*> m_manifoldsPtr;
    pub manifolds: Vec<PersistentManifold>,
    // near_callback: NearCallback,
    // btPoolAllocator* m_collisionAlgorithmPoolAllocator;
    // btPoolAllocator* m_persistentManifoldPoolAllocator;
    // btCollisionAlgorithmCreateFunc* m_doubleDispatchContactPoints[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
    // btCollisionAlgorithmCreateFunc* m_doubleDispatchClosestPoints[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
    // btCollisionConfiguration* m_collisionConfiguration;
}

impl CollisionDispatcher {
    fn needs_collision(&self, body0: &CollisionObject, body1: &CollisionObject) -> bool {
        (body0.is_active() || body1.is_active())
            && body0.check_collide_with(body1)
            && body1.check_collide_with(body0)
    }

    fn find_algorithm(
        &self,
        col_obj_0: Rc<RefCell<CollisionObject>>,
        col_obj_1: Rc<RefCell<CollisionObject>>,
    ) -> impl CollisionAlgorithm + Sized {
        let shape0 = col_obj_0
            .borrow()
            .get_collision_shape()
            .unwrap()
            .borrow()
            .get_shape_type();
        let shape1 = col_obj_1
            .borrow()
            .get_collision_shape()
            .unwrap()
            .borrow()
            .get_shape_type();

        match shape0 {
            BroadphaseNativeTypes::StaticPlaneProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => {
                    ConvexPlaneCollisionAlgorithm::new(col_obj_1, col_obj_0, true)
                }
                shape1 => todo!("shape1: {shape1:?}"),
            },
            BroadphaseNativeTypes::SphereShapeProxytype => match shape1 {
                BroadphaseNativeTypes::StaticPlaneProxytype => {
                    ConvexPlaneCollisionAlgorithm::new(col_obj_0, col_obj_1, false)
                }
                shape1 => todo!("shape1: {shape1:?}"),
            },
            shape0 => todo!("shape0: {shape0:?}"),
        }
    }

    fn near_callback(&mut self, collision_pair: &BroadphasePair, dispatch_info: &DispatcherInfo) {
        let proxy0 = collision_pair.proxy0.borrow();
        let proxy1 = collision_pair.proxy1.borrow();

        let col_obj_0 = proxy0
            .broadphase_proxy
            .client_object
            .as_ref()
            .unwrap()
            .borrow();
        let col_obj_1 = proxy1
            .broadphase_proxy
            .client_object
            .as_ref()
            .unwrap()
            .borrow();

        if !self.needs_collision(&col_obj_0, &col_obj_1) {
            return;
        }

        let mut algorithm = self.find_algorithm(
            proxy0
                .broadphase_proxy
                .client_object
                .as_ref()
                .unwrap()
                .clone(),
            proxy1
                .broadphase_proxy
                .client_object
                .as_ref()
                .unwrap()
                .clone(),
        );

        debug_assert_eq!(dispatch_info.dispatch_func, DispatchFunc::DispatchDiscrete);
        algorithm.process_collision(&col_obj_0, &col_obj_1);

        self.manifolds.push(algorithm.into_manifold());
    }

    pub fn dispatch_all_collision_pairs(
        &mut self,
        pair_cache: &mut dyn OverlappingPairCache,
        dispatch_info: &DispatcherInfo,
    ) {
        let mut collision_callback = CollisionPairCallback {
            dispatch_info,
            dispatcher: self,
        };

        pair_cache.process_all_overlapping_pairs(&mut collision_callback);
    }
}
