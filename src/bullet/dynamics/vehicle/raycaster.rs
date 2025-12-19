use glam::Vec3A;

use crate::bullet::{
    collision::dispatch::{
        collision_object::CollisionObject,
        collision_world::{ClosestRayResultCallback, RayResultCallback},
    },
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

pub struct VehicleRaycasterResult<'a> {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    pub rigid_body: &'a RigidBody,
}

pub struct VehicleRaycaster {
    added_filter_mask: u8,
}

impl VehicleRaycaster {
    pub const fn new(added_filter_mask: u8) -> Self {
        Self { added_filter_mask }
    }

    pub fn cast_ray<'a>(
        &self,
        collision_world: &'a DiscreteDynamicsWorld,
        from: Vec3A,
        to: Vec3A,
        ignore_obj: &CollisionObject,
    ) -> Option<VehicleRaycasterResult<'a>> {
        let mut ray_callback = ClosestRayResultCallback::new(from, to, ignore_obj);
        ray_callback.base.collision_filter_group |= self.added_filter_mask;
        collision_world
            .dynamics_world
            .collision_world
            .ray_test(from, to, &mut ray_callback);

        if ray_callback.has_hit()
            && let Some(co_index) = ray_callback.base.collision_object_index
        {
            let rb = &collision_world
                .dynamics_world
                .collision_world
                .collision_objects[co_index];
            if rb.collision_object.has_contact_response() {
                Some(VehicleRaycasterResult {
                    rigid_body: rb,
                    hit_point_in_world: ray_callback.hit_point_world,
                    hit_normal_in_world: ray_callback.hit_normal_world.normalize(),
                })
            } else {
                None
            }
        } else {
            None
        }
    }
}
