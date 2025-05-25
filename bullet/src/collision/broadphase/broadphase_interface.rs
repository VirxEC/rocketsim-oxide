use super::broadphase_proxy::{BroadphaseNativeTypes, BroadphaseProxy};
use crate::collision::dispatch::collision_object::CollisionObject;
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

pub trait BroadphaseInterface {
    fn create_proxy(
        &mut self,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
        shape_type: BroadphaseNativeTypes,
        // user_ptr: *mut std::ffi::c_void,
        user_ptr: Rc<RefCell<CollisionObject>>,
        collision_filter_group: i32,
        collision_filter_mask: i32,
        // dispatcher: &CollisionDispatcher,
    ) -> Rc<RefCell<BroadphaseProxy>>;
}
