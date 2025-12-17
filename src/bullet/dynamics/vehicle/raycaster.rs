use std::{cell::RefCell, rc::Rc};

use glam::Vec3A;

use crate::bullet::{
    collision::dispatch::{
        collision_object::CollisionObject,
        collision_world::{ClosestRayResultCallback, RayResultCallback},
    },
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

pub struct VehicleRaycasterResult {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    // pub dist_fraction: f32,
    pub rigid_body: Rc<RefCell<RigidBody>>,
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
        collision_world: &DiscreteDynamicsWorld,
        from: Vec3A,
        to: Vec3A,
        ignore_obj: &CollisionObject,
    ) -> Option<VehicleRaycasterResult> {
        let mut ray_callback = ClosestRayResultCallback::new(from, to, ignore_obj);
        ray_callback.base.collision_filter_group |= self.added_filter_mask;
        collision_world
            .dynamics_world
            .collision_world
            .ray_test(from, to, &mut ray_callback);

        if ray_callback.has_hit()
            && let Some(co_index) = ray_callback.base.collision_object_index
        {
            let rb_ref = &collision_world
                .dynamics_world
                .collision_world
                .collision_objects[co_index];
            let rb = rb_ref.borrow();
            if rb.collision_object.has_contact_response() {
                Some(VehicleRaycasterResult {
                    rigid_body: rb_ref.clone(),
                    hit_point_in_world: ray_callback.hit_point_world,
                    hit_normal_in_world: ray_callback.hit_normal_world.normalize(),
                    // dist_fraction: ray_callback.base.closest_hit_fraction,
                })
            } else {
                None
            }
        } else {
            None
        }
    }
}
