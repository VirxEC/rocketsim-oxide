use crate::bullet::collision::dispatch::{
    collision_object::CollisionObject,
    collision_world::{ClosestRayResultCallback, CollisionWorld, RayResultCallback},
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

pub struct VehicleRaycasterResult {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    pub dist_fraction: f32,
}

pub struct VehicleRaycaster {
    added_filter_mask: i32,
}

impl VehicleRaycaster {
    pub const fn new(added_filter_mask: i32) -> Self {
        Self { added_filter_mask }
    }

    pub fn cast_ray(
        &self,
        collision_world: &CollisionWorld,
        from: Vec3A,
        to: Vec3A,
        ignore_obj: &Rc<RefCell<CollisionObject>>,
    ) -> Option<VehicleRaycasterResult> {
        let mut ray_callback = ClosestRayResultCallback::new(from, to, ignore_obj);
        ray_callback.base.collision_filter_group |= self.added_filter_mask;
        collision_world.ray_test(from, to, &ray_callback);

        if ray_callback.has_hit() {
            todo!("vehicle raycast hit");
        } else {
            None
        }
    }
}
