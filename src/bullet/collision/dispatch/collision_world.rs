use super::{
    collision_dispatcher::CollisionDispatcher,
    collision_object::{CollisionObject, CollisionObjectTypes},
};
use crate::bullet::collision::{
    broadphase::{dispatcher::DispatcherInfo, rs_broadphase::RsBroadphase},
    narrowphase::persistent_manifold::CONTACT_BREAKING_THRESHOLD,
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

pub struct CollisionWorld {
    pub collision_objects: Vec<Rc<RefCell<CollisionObject>>>,
    pub dispatcher1: CollisionDispatcher,
    pub dispatcher_info: DispatcherInfo,
    broadphase_pair_cache: RsBroadphase,
    force_update_all_aabbs: bool,
}

impl CollisionWorld {
    pub fn new(dispatcher: CollisionDispatcher, pair_cache: RsBroadphase) -> Self {
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

        drop(shape);
        drop(obj);

        let proxy = self.broadphase_pair_cache.create_proxy(
            aabb_min,
            aabb_max,
            object.clone(),
            filter_group,
            filter_mask,
        );

        object.borrow_mut().set_broadphase_handle(proxy);
        self.collision_objects.push(object);
    }

    fn update_single_aabb(&mut self, col_obj_idx: usize) {
        let col_obj = self.collision_objects[col_obj_idx].borrow();
        let (mut min_aabb, mut max_aabb) = col_obj
            .get_collision_shape()
            .as_ref()
            .unwrap()
            .borrow()
            .get_aabb(col_obj.get_world_transform());

        let contact_threshold = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);
        min_aabb -= contact_threshold;
        max_aabb += contact_threshold;

        if self.dispatcher_info.use_continuous
            && col_obj.internal_type == CollisionObjectTypes::RigidBody as i32
            && !col_obj.is_static_or_kinematic_object()
        {
            let (mut min_aabb_2, mut max_aabb_2) = col_obj
                .get_collision_shape()
                .as_ref()
                .unwrap()
                .borrow()
                .get_aabb(&col_obj.interpolation_world_transform);
            min_aabb_2 -= contact_threshold;
            max_aabb_2 += contact_threshold;

            min_aabb = min_aabb.min(min_aabb_2);
            max_aabb = max_aabb.max(max_aabb_2);
        }

        if col_obj.is_static_object() || (max_aabb - min_aabb).length_squared() < 1e12 {
            self.broadphase_pair_cache.set_aabb(
                col_obj.get_broadphase_handle().unwrap(),
                min_aabb,
                max_aabb,
            );
        } else {
            unreachable!()
        }
    }

    fn update_aabbs(&mut self) {
        for i in 0..self.collision_objects.len() {
            let col_obj = &self.collision_objects[i];
            debug_assert!(col_obj.borrow().get_world_array_index() as usize == i);

            if self.force_update_all_aabbs || col_obj.borrow().is_active() {
                self.update_single_aabb(i);
            }
        }
    }

    pub fn perform_discrete_collision_detection(&mut self) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();

        self.dispatcher1
            .dispatch_all_collision_pairs(&mut self.broadphase_pair_cache);
    }
}
