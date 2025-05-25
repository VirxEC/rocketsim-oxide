use super::{collision_dispatcher::CollisionDispatcher, collision_object::CollisionObject};
use crate::collision::broadphase::{
    broadphase_interface::BroadphaseInterface, dispatcher::DispatcherInfo,
};
use std::{cell::RefCell, rc::Rc};

pub struct CollisionWorld {
    collision_objects: Vec<Rc<RefCell<CollisionObject>>>,
    dispatcher1: CollisionDispatcher,
    dispatcher_info: DispatcherInfo,
    broadphase_pair_cache: Box<dyn BroadphaseInterface>,
    force_update_all_aabbs: bool,
}

impl CollisionWorld {
    pub fn new(dispatcher: CollisionDispatcher, pair_cache: Box<dyn BroadphaseInterface>) -> Self {
        Self {
            collision_objects: Vec::new(),
            dispatcher1: dispatcher,
            dispatcher_info: DispatcherInfo::default(),
            broadphase_pair_cache: pair_cache,
            force_update_all_aabbs: true,
        }
    }

    pub fn add_collision_object(
        &mut self,
        object: Rc<RefCell<CollisionObject>>,
        filter_group: i32,
        filter_mask: i32,
    ) {
        object
            .borrow_mut()
            .set_world_array_index(self.collision_objects.len() as i32);

        let obj = object.borrow();
        let trans = obj.get_world_transform();

        let shape = obj.get_collision_shape().unwrap().borrow_mut();
        let (aabb_min, aabb_max) = shape.get_aabb(trans);
        let shape_type = shape.get_shape_type();

        drop(shape);
        drop(obj);

        let proxy = self.broadphase_pair_cache.create_proxy(
            aabb_min,
            aabb_max,
            shape_type,
            object.clone(),
            filter_group,
            filter_mask,
            // &self.dispatcher1,
        );

        object.borrow_mut().set_broadphase_handle(proxy);
        self.collision_objects.push(object);
    }
}
