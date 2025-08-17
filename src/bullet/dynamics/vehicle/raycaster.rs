use crate::bullet::{
    collision::dispatch::{
        collision_object::CollisionObject,
        collision_world::{ClosestRayResultCallback, RayResultCallback},
    },
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

pub struct VehicleRaycasterResult {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    pub dist_fraction: f32,
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
            && let Some(co_ref) = ray_callback.base.collision_object.as_ref()
        {
            let co = co_ref.borrow();
            if co.has_contact_response() {
                let rigid_body = if co.is_static_object() {
                    &collision_world.static_rigid_bodies
                } else {
                    &collision_world.non_static_rigid_bodies
                }[co.get_rigid_body_world_index()]
                .clone();

                Some(VehicleRaycasterResult {
                    rigid_body,
                    hit_point_in_world: ray_callback.hit_point_world,
                    hit_normal_in_world: ray_callback.hit_normal_world.normalize(),
                    dist_fraction: ray_callback.base.closest_hit_fraction,
                })
            } else {
                None
            }
        } else {
            None
        }
    }
}
