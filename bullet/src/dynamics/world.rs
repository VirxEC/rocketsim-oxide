use super::constraint_solver::contact_solver_info::ContactSolverInfo;
use crate::collision::{
    broadphase::rs_broadphase::RsBroadphase,
    dispatch::{collision_dispatcher::CollisionDispatcher, collision_world::CollisionWorld},
};

type InternalTickCallback = fn(world: &mut DynamicsWorld, time_step: f32);

pub struct DynamicsWorld {
    pub collision_world: CollisionWorld,
    internal_tick_callback: Option<InternalTickCallback>,
    internal_pre_tick_callback: Option<InternalTickCallback>,
    // void* m_worldUserInfo;
    pub solver_info: ContactSolverInfo,
}

impl DynamicsWorld {
    pub fn new(dispatcher: CollisionDispatcher, broadphase: RsBroadphase) -> Self {
        Self {
            collision_world: CollisionWorld::new(dispatcher, broadphase),
            internal_tick_callback: None,
            internal_pre_tick_callback: None,
            solver_info: ContactSolverInfo::default(),
        }
    }
}
