use super::{
    broadphase_proxy::BroadphaseNativeTypes, overlapping_pair_cache::OverlappingPairCache,
    rs_broadphase::RsBroadphaseProxy,
};
use crate::collision::dispatch::{
    collision_dispatcher::CollisionDispatcher, collision_object::CollisionObject,
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

pub trait BroadphaseInterface {
    fn set_aabb(
        &mut self,
        proxy: &Rc<RefCell<RsBroadphaseProxy>>,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
    );

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
    ) -> Rc<RefCell<RsBroadphaseProxy>>;

    fn calculate_overlapping_pairs(&mut self);

    fn get_overlapping_pair_cache(&mut self) -> &mut dyn OverlappingPairCache;
}
