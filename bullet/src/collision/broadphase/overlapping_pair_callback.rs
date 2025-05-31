use super::rs_broadphase::RsBroadphaseProxy;
use std::{cell::RefCell, rc::Rc};

pub trait OverlappingPairCallback {
    fn add_overlapping_pair(
        &mut self,
        proxy0: &Rc<RefCell<RsBroadphaseProxy>>,
        proxy1: &Rc<RefCell<RsBroadphaseProxy>>,
    );
}
