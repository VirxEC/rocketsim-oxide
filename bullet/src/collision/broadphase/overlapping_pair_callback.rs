use super::rs_broadphase::RsBroadphaseProxy;
use crate::collision::dispatch::collision_dispatcher::CollisionDispatcher;
use std::{cell::RefCell, rc::Rc};

pub trait OverlappingPairCallback {
    fn add_overlapping_pair(
        &mut self,
        proxy0: &Rc<RefCell<RsBroadphaseProxy>>,
        proxy1: &Rc<RefCell<RsBroadphaseProxy>>,
    );

    fn remove_overlapping_pair(
        &mut self,
        proxy0: &Rc<RefCell<RsBroadphaseProxy>>,
        proxy1: &Rc<RefCell<RsBroadphaseProxy>>,
        dispatcher: &mut CollisionDispatcher,
    );
}
