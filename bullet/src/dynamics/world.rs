use super::constraint_solver::contact_solver_info::ContactSolverInfo;
use crate::collision::{
    broadphase::broadphase_interface::BroadphaseInterface,
    dispatch::{collision_dispatcher::CollisionDispatcher, collision_world::CollisionWorld},
};

type InternalTickCallback = fn(world: &mut DynamicsWorld, time_step: f32);

pub enum DynamicsWorldType {
    SimpleDynamicsWorld = 1,
    DiscreteDynamicsWorld = 2,
    ContinuousDynamicsWorld = 3,
    SoftRigidDynamicsWorld = 4,
    GpuDynamicsWorld = 5,
    SoftMultibodyDynamicsWorld = 6,
    DeformableMultibodyDynamicsWorld = 7,
}

pub struct DynamicsWorld {
    pub collision_world: CollisionWorld,
    internal_tick_callback: Option<InternalTickCallback>,
    internal_pre_tick_callback: Option<InternalTickCallback>,
    // void* m_worldUserInfo;
    pub solver_info: ContactSolverInfo,
}

impl DynamicsWorld {
    pub fn new(dispatcher: CollisionDispatcher, broadphase: Box<dyn BroadphaseInterface>) -> Self {
        Self {
            collision_world: CollisionWorld::new(dispatcher, broadphase),
            internal_tick_callback: None,
            internal_pre_tick_callback: None,
            solver_info: ContactSolverInfo::default(),
        }
    }
}
