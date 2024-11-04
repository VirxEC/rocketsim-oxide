use std::{cell::RefCell, rc::Rc};

use crate::bullet::{
    btcollision::{
        broadphase::proxy::CollisionFilterGroups,
        dispatch::{collision_object::ActivationState, collision_world::CollisionWorld},
    },
    btdynamics::constraint_solver::contact_solver_info::ContactSolverInfo,
    linear_math::vector3::Vector3,
};

use super::rigid_body::RigidBody;

#[derive(Default)]
pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    gravity: Vector3,
    solver_info: ContactSolverInfo,
    non_static_rigid_bodies: Vec<Rc<RefCell<RigidBody>>>,
}

impl DiscreteDynamicsWorld {
    pub fn set_gravity(&mut self, gravity: Vector3) {
        self.gravity = gravity;

        self.non_static_rigid_bodies
            .iter()
            .filter(|body| {
                body.borrow().collision_object.is_active() // && !(body_r.collision_object.collision_flags & CollisionFlags::DisableWorldGravity as i32 != 0)
            })
            .for_each(|body| body.borrow_mut().set_gravity(gravity))
    }

    fn add_collision_object(
        &mut self,
        collision_object: Rc<RefCell<RigidBody>>,
        collision_filter_group: i32,
        collision_filter_mask: i32,
    ) {
        self.collision_world.add_collision_object(
            collision_object,
            collision_filter_group,
            collision_filter_mask,
        )
    }

    pub fn add_rigid_body(&mut self, body: Rc<RefCell<RigidBody>>) {
        let mut body_w = body.borrow_mut();

        let is_dynamic = !body_w.is_static_or_kinematic_object();
        if is_dynamic {
            body_w.set_gravity(self.gravity);
        }

        if body_w.collision_object.has_collision_shape() {
            if body_w.is_static_object() {
                self.non_static_rigid_bodies.push(body.clone());
            } else {
                body_w
                    .collision_object
                    .set_activation_state(ActivationState::IslandSleeping);
            }

            drop(body_w);

            let collision_filter_group = if is_dynamic {
                CollisionFilterGroups::DefaultFilter as i32
            } else {
                CollisionFilterGroups::StaticFilter as i32
            };

            let collision_filter_mask = if is_dynamic {
                CollisionFilterGroups::AllFilter as i32
            } else {
                CollisionFilterGroups::AllFilter as i32 ^ CollisionFilterGroups::StaticFilter as i32
            };

            self.add_collision_object(body, collision_filter_group, collision_filter_mask)
        }
    }
}
