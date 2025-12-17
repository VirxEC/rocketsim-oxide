use std::{cell::RefCell, rc::Rc};

use glam::Vec3A;

use super::{
    constraint_solver::sequential_impulse_constraint_solver::SequentialImpulseConstraintSolver,
    rigid_body::{RigidBody, RigidBodyFlags},
    world::DynamicsWorld,
};
use crate::bullet::collision::{
    broadphase::{broadphase_proxy::CollisionFilterGroups, rs_broadphase::RsBroadphase},
    dispatch::{
        collision_dispatcher::CollisionDispatcher,
        collision_object::{ACTIVE_TAG, ISLAND_SLEEPING, WANTS_DEACTIVATION},
    },
    narrowphase::persistent_manifold::ContactAddedCallback,
};

pub struct DiscreteDynamicsWorld {
    pub dynamics_world: DynamicsWorld,
    solver: SequentialImpulseConstraintSolver,
    non_static_rigid_bodies: Vec<usize>,
    gravity: Vec3A,
    apply_speculative_contact_restitution: bool,
}

impl DiscreteDynamicsWorld {
    #[must_use]
    pub fn new(
        dispatcher: CollisionDispatcher,
        pair_cache: RsBroadphase,
        constraint_solver: SequentialImpulseConstraintSolver,
    ) -> Self {
        Self {
            dynamics_world: DynamicsWorld::new(dispatcher, pair_cache),
            solver: constraint_solver,
            gravity: Vec3A::new(0.0, -10.0, 0.0),
            apply_speculative_contact_restitution: false,
            non_static_rigid_bodies: Vec::new(),
        }
    }

    pub const fn set_gravity(&mut self, gravity: Vec3A) {
        self.gravity = gravity;
    }

    fn add_collision_object(
        &mut self,
        body: Rc<RefCell<RigidBody>>,
        group: i32,
        mask: i32,
    ) -> usize {
        self.dynamics_world
            .collision_world
            .add_collision_object(body, group, mask)
    }

    pub fn add_rigid_body_default(&mut self, body: Rc<RefCell<RigidBody>>) {
        if !body
            .borrow()
            .collision_object
            .is_static_or_kinematic_object()
            && body.borrow().get_flags() & RigidBodyFlags::DisableWorldGravity as i32 == 0
        {
            body.borrow_mut().set_gravity(self.gravity);
        }

        if body
            .borrow()
            .collision_object
            .get_collision_shape()
            .is_some()
        {
            let (group, mask) = if body
                .borrow()
                .collision_object
                .is_static_or_kinematic_object()
            {
                (
                    CollisionFilterGroups::StaticFilter as i32,
                    CollisionFilterGroups::AllFilter as i32
                        ^ CollisionFilterGroups::StaticFilter as i32,
                )
            } else {
                (
                    CollisionFilterGroups::DefaultFilter as i32,
                    CollisionFilterGroups::AllFilter as i32,
                )
            };

            let rb_index = self.add_collision_object(body, group, mask);

            let mut rb =
                self.dynamics_world.collision_world.collision_objects[rb_index].borrow_mut();
            if rb.collision_object.is_static_object() {
                rb.collision_object.set_activation_state(ISLAND_SLEEPING);
            } else {
                self.non_static_rigid_bodies.push(rb_index);
            }
        }
    }

    pub fn add_rigid_body(&mut self, body: Rc<RefCell<RigidBody>>, group: i32, mask: i32) {
        if !body
            .borrow()
            .collision_object
            .is_static_or_kinematic_object()
            && body.borrow().get_flags() & RigidBodyFlags::DisableWorldGravity as i32 == 0
        {
            body.borrow_mut().set_gravity(self.gravity);
        }

        if body
            .borrow()
            .collision_object
            .get_collision_shape()
            .is_some()
        {
            let rb_index = self.add_collision_object(body, group, mask);

            let mut rb =
                self.dynamics_world.collision_world.collision_objects[rb_index].borrow_mut();
            if rb.collision_object.is_static_object() {
                rb.collision_object.set_activation_state(ISLAND_SLEEPING);
            } else {
                self.non_static_rigid_bodies.push(rb_index);
            }
        }
    }

    fn apply_gravity(&self) {
        for &body in &self.non_static_rigid_bodies {
            let mut body = self.dynamics_world.collision_world.collision_objects[body].borrow_mut();
            if body.collision_object.is_active() {
                body.apply_gravity();
            }
        }
    }

    fn predict_unconstraint_motion(&self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let mut body = self.dynamics_world.collision_world.collision_objects[body].borrow_mut();
            debug_assert!(!body.collision_object.is_static_or_kinematic_object());

            body.apply_damping(time_step);
            let predicted_transform = body.predict_integration_transform(time_step);
            body.collision_object.interpolation_world_transform = predicted_transform;
        }
    }

    fn calculation_simulation_islands(&self) {
        for manifold in &self.dynamics_world.collision_world.dispatcher1.manifolds {
            let bodies = &self.dynamics_world.collision_world.collision_objects;
            let mut body0 = bodies[manifold.body0_idx].borrow_mut();
            let mut body1 = bodies[manifold.body1_idx].borrow_mut();

            if body0.collision_object.get_activation_state() != ISLAND_SLEEPING
                || body1.collision_object.get_activation_state() != ISLAND_SLEEPING
            {
                if body0.collision_object.is_kinematic_object()
                    && body0.collision_object.get_activation_state() != ISLAND_SLEEPING
                {
                    body1.collision_object.activate();
                }

                if body1.collision_object.is_kinematic_object()
                    && body1.collision_object.get_activation_state() != ISLAND_SLEEPING
                {
                    body0.collision_object.activate();
                }

                debug_assert!(
                    !body0.collision_object.is_static_or_kinematic_object()
                        || !body1.collision_object.is_static_or_kinematic_object()
                );
            }
        }
    }

    fn solve_constraints(&mut self) {
        self.solver.solve_group(
            &self.dynamics_world.collision_world.collision_objects,
            &self.non_static_rigid_bodies,
            &mut self.dynamics_world.collision_world.dispatcher1.manifolds,
            &self.dynamics_world.solver_info,
        );
    }

    fn integrate_transforms_internal(&self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let mut body = self.dynamics_world.collision_world.collision_objects[body].borrow_mut();
            body.collision_object.hit_fraction = 1.0;

            debug_assert!(!body.collision_object.is_static_or_kinematic_object());
            if !body.collision_object.is_active() {
                continue;
            }

            let predicted_trans = body.predict_integration_transform(time_step);
            body.set_center_of_mass_transform(predicted_trans);
        }
    }

    fn integrate_transforms(&self, time_step: f32) {
        if !self.non_static_rigid_bodies.is_empty() {
            self.integrate_transforms_internal(time_step);
        }

        debug_assert!(!self.apply_speculative_contact_restitution);
    }

    fn update_activation_state(&self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let mut body = self.dynamics_world.collision_world.collision_objects[body].borrow_mut();
            body.update_deactivation(time_step);

            if body.wants_sleeping() {
                if body.collision_object.is_static_or_kinematic_object() {
                    body.collision_object.set_activation_state(ISLAND_SLEEPING);
                } else if body.collision_object.activation_state_1 == ACTIVE_TAG {
                    body.collision_object
                        .set_activation_state(WANTS_DEACTIVATION);
                } else if body.collision_object.activation_state_1 == ISLAND_SLEEPING {
                    body.set_angular_velocity(Vec3A::ZERO);
                    body.set_linear_velocity(Vec3A::ZERO);
                }
            } else {
                body.collision_object.set_activation_state(ACTIVE_TAG);
            }
        }
    }

    fn clear_forces(&self) {
        for &body in &self.non_static_rigid_bodies {
            self.dynamics_world.collision_world.collision_objects[body]
                .borrow_mut()
                .clear_forces();
        }
    }

    fn internal_single_step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.predict_unconstraint_motion(time_step);

        let dispatcher_info = &mut self.dynamics_world.collision_world.dispatcher_info;
        dispatcher_info.time_step = time_step;
        dispatcher_info.step_count = 0;

        self.dynamics_world
            .collision_world
            .perform_discrete_collision_detection(contact_added_callback);

        self.calculation_simulation_islands();

        self.dynamics_world.solver_info.time_step = time_step;

        self.solve_constraints();
        self.integrate_transforms(time_step);
        self.update_activation_state(time_step);
    }

    pub fn step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.apply_gravity();
        self.internal_single_step_simulation(time_step, contact_added_callback);

        self.clear_forces();
    }
}
