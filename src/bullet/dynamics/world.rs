use super::constraint_solver::contact_solver_info::ContactSolverInfo;
use crate::bullet::collision::{
    broadphase::rs_broadphase::RsBroadphase,
    dispatch::{collision_dispatcher::CollisionDispatcher, collision_world::CollisionWorld},
};

pub struct DynamicsWorld {
    pub collision_world: CollisionWorld,
    pub solver_info: ContactSolverInfo,
}

impl DynamicsWorld {
    pub fn new(dispatcher: CollisionDispatcher, broadphase: RsBroadphase) -> Self {
        Self {
            collision_world: CollisionWorld::new(dispatcher, broadphase),
            solver_info: ContactSolverInfo::default(),
        }
    }
}
