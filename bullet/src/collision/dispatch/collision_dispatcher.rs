use super::{
    collision_object::CollisionObject,
    convex_concave_collision_algorithm::ConvexConcaveCollisionAlgorithm,
    convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
};
use crate::collision::{
    broadphase::{
        broadphase_proxy::{BroadphaseNativeTypes, BroadphasePair},
        collision_algorithm::CollisionAlgorithm,
        dispatcher::DispatcherInfo,
        overlapping_pair_cache::OverlappingPairCache,
    },
    narrowphase::persistent_manifold::PersistentManifold,
};
use std::{cell::RefCell, rc::Rc};

pub type NearCallback = fn(
    collision_pair: &BroadphasePair,
    dispatcher: &mut CollisionDispatcher,
    dispatch_info: &DispatcherInfo,
);

const MAX_BROADPHASE_COLLISION_TYPES: usize =
    BroadphaseNativeTypes::MaxBroadphaseCollisionTypes as usize;

enum Algorithms {
    ConvexPlane(ConvexPlaneCollisionAlgorithm),
    ConvexConcave(ConvexConcaveCollisionAlgorithm),
}

impl Algorithms {
    fn new_convex_plane(
        convex_obj: Rc<RefCell<CollisionObject>>,
        plane_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
    ) -> Self {
        Self::ConvexPlane(ConvexPlaneCollisionAlgorithm::new(
            convex_obj, plane_obj, is_swapped,
        ))
    }

    fn new_convex_concave(
        convex_obj: Rc<RefCell<CollisionObject>>,
        concave_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
    ) -> Self {
        Self::ConvexConcave(ConvexConcaveCollisionAlgorithm::new(
            convex_obj,
            concave_obj,
            is_swapped,
        ))
    }
}

impl CollisionAlgorithm for Algorithms {
    fn into_manifold(self) -> PersistentManifold {
        match self {
            Self::ConvexPlane(alg) => alg.into_manifold(),
            Self::ConvexConcave(alg) => alg.into_manifold(),
        }
    }

    fn process_collision(&mut self, body0: &CollisionObject, body1: &CollisionObject) {
        match self {
            Self::ConvexPlane(alg) => alg.process_collision(body0, body1),
            Self::ConvexConcave(alg) => alg.process_collision(body0, body1),
        }
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
    fn find_algorithm(
        &self,
        col_obj_0: Rc<RefCell<CollisionObject>>,
        col_obj_1: Rc<RefCell<CollisionObject>>,
    ) -> Algorithms {
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
                    Algorithms::new_convex_plane(col_obj_1, col_obj_0, true)
                }
                shape1 => todo!("shape1: {shape1:?}"),
            },
            BroadphaseNativeTypes::SphereShapeProxytype => match shape1 {
                BroadphaseNativeTypes::StaticPlaneProxytype => {
                    Algorithms::new_convex_plane(col_obj_0, col_obj_1, false)
                }
                BroadphaseNativeTypes::TriangleMeshShapeProxytype => {
                    Algorithms::new_convex_concave(col_obj_0, col_obj_1, false)
                }
                shape1 => todo!("shape1: {shape1:?}"),
            },
            BroadphaseNativeTypes::TriangleMeshShapeProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => {
                    Algorithms::new_convex_concave(col_obj_1, col_obj_0, true)
                }
                shape1 => todo!("shape1: {shape1:?}"),
            },
            shape0 => todo!("shape0: {shape0:?}"),
        }
    }

    pub fn near_callback(&mut self, collision_pair: &BroadphasePair) {
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

        if !col_obj_0.is_active() && !col_obj_1.is_active() {
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

        algorithm.process_collision(&col_obj_0, &col_obj_1);

        self.manifolds.push(algorithm.into_manifold());
    }

    pub fn dispatch_all_collision_pairs(&mut self, pair_cache: &mut dyn OverlappingPairCache) {
        pair_cache.process_all_overlapping_pairs(self);
    }
}
