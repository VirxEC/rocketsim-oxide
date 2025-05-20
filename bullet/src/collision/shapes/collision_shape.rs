use crate::collision::broadphase::broadphase_proxy::BroadphaseNativeTypes;
use glam::{Affine3A, Vec3A};
use std::cell::RefCell;

pub struct CollisionShape {
    pub shape_type: BroadphaseNativeTypes,
    // pub user_pointer: *mut c_void,
    pub user_index: i32,
    pub user_index_2: i32,
    pub aabb_cached: RefCell<bool>,
    pub aabb_min_cache: RefCell<Vec3A>,
    pub aabb_max_cache: RefCell<Vec3A>,
    pub aabb_cache_trans: RefCell<Affine3A>,
}

impl Default for CollisionShape {
    fn default() -> Self {
        Self {
            shape_type: BroadphaseNativeTypes::InvalidShapeProxytype,
            user_index: -1,
            user_index_2: -1,
            aabb_cached: RefCell::default(),
            aabb_min_cache: RefCell::default(),
            aabb_max_cache: RefCell::default(),
            aabb_cache_trans: RefCell::default(),
        }
    }
}
