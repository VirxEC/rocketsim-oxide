use super::collision_configuration::CollisionConfiguration;
use crate::collision::narrowphase::{
    convex_penetration_depth_solver::ConvexPenetrationDepthSolver,
    gjk_epa_penetration_depth_solver::GjkEpaPenetrationDepthSolver,
};
use std::rc::Rc;

pub struct DefaultCollisionConstructionInfo {
    // btPoolAllocator* m_persistentManifoldPool;
    // btPoolAllocator* m_collisionAlgorithmPool;
    pub default_max_persistent_manifold_pool_size: usize,
    pub default_max_collision_algorithm_pool_size: usize,
    pub custom_collision_algorithm_max_element_size: usize,
    pub use_epa_penetration_algorithm: bool,
}

impl Default for DefaultCollisionConstructionInfo {
    fn default() -> Self {
        Self {
            default_max_persistent_manifold_pool_size: 4096,
            default_max_collision_algorithm_pool_size: 4096,
            custom_collision_algorithm_max_element_size: 0,
            use_epa_penetration_algorithm: true,
        }
    }
}

pub struct DefaultCollisionConfiguration {
    persistent_manifold_pool_size: usize,

    // btPoolAllocator* m_persistentManifoldPool;
    // bool m_ownsPersistentManifoldPool;

    // btPoolAllocator* m_collisionAlgorithmPool;
    // bool m_ownsCollisionAlgorithmPool;
    pd_solver: Rc<dyn ConvexPenetrationDepthSolver>,
    // btCollisionAlgorithmCreateFunc* m_convexConvexCreateFunc;
    // btCollisionAlgorithmCreateFunc* m_convexConcaveCreateFunc;
    // btCollisionAlgorithmCreateFunc* m_swappedConvexConcaveCreateFunc;
    // btCollisionAlgorithmCreateFunc* m_compoundCreateFunc;
    // btCollisionAlgorithmCreateFunc* m_compoundCompoundCreateFunc;

    // btCollisionAlgorithmCreateFunc* m_swappedCompoundCreateFunc;
    // btCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
    // btCollisionAlgorithmCreateFunc* m_sphereSphereCF;
    // btCollisionAlgorithmCreateFunc* m_sphereBoxCF;
    // btCollisionAlgorithmCreateFunc* m_boxSphereCF;

    // btCollisionAlgorithmCreateFunc* m_boxBoxCF;
    // btCollisionAlgorithmCreateFunc* m_sphereTriangleCF;
    // btCollisionAlgorithmCreateFunc* m_triangleSphereCF;
    // btCollisionAlgorithmCreateFunc* m_planeConvexCF;
    // btCollisionAlgorithmCreateFunc* m_convexPlaneCF;
}

impl DefaultCollisionConfiguration {
    pub fn new(construction_info: DefaultCollisionConstructionInfo) -> Self {
        let pd_solver = if construction_info.use_epa_penetration_algorithm {
            Rc::new(GjkEpaPenetrationDepthSolver {})
        } else {
            unimplemented!()
        };

        Self {
            persistent_manifold_pool_size: 0,
            pd_solver,
        }
    }
}

impl CollisionConfiguration for DefaultCollisionConfiguration {}
