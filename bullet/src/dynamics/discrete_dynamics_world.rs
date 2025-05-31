use super::{
    constraint_solver::{
        contact_solver_info::ContactSolverInfo,
        sequential_impulse_constraint_solver::SequentialImpulseConstraintSolver,
        typed_constraint::TypedConstraint,
    },
    rigid_body::{RigidBody, RigidBodyFlags},
    world::DynamicsWorld,
};
use crate::collision::{
    broadphase::{
        broadphase_interface::BroadphaseInterface, broadphase_proxy::CollisionFilterGroups,
    },
    dispatch::{
        collision_dispatcher::CollisionDispatcher,
        collision_object::{
            ACTIVE_TAG, CollisionObject, DISABLE_DEACTIVATION, ISLAND_SLEEPING, WANTS_DEACTIVATION,
        },
        simulation_island_manager::{IslandCallback, SimulationIslandManager},
    },
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

struct InplaceSolverIslandCallback {
    solver_info: Option<ContactSolverInfo>,
    solver: SequentialImpulseConstraintSolver,
    // btTypedConstraint **m_sortedConstraints;
    // num_constraints: usize,
    // bodies: Vec<CollisionObject>,
    // manifolds: Vec<PersistentManifold>,
    // constraints: Vec<TypedConstraint>,
}

impl InplaceSolverIslandCallback {
    fn new(solver: SequentialImpulseConstraintSolver) -> Self {
        Self {
            solver,
            solver_info: None,
            // num_constraints: 0,
            // bodies: Vec::new(),
            // manifolds: Vec::new(),
            // constraints: Vec::new(),
        }
    }
}

impl IslandCallback for InplaceSolverIslandCallback {}

pub struct DiscreteDynamicsWorld {
    pub dynamics_world: DynamicsWorld,
    // sorted_constraints: Vec<TypedConstraint>,
    solver_island_callback: InplaceSolverIslandCallback,
    // island_manager: SimulationIslandManager,
    // constraints: Vec<TypedConstraint>,
    non_static_rigid_bodies: Vec<Rc<RefCell<RigidBody>>>,
    gravity: Vec3A,
    local_time: f32,
    fixed_time_step: f32,
    // bool m_ownsIslandManager;
    // bool m_ownsConstraintSolver;
    synchronize_all_motion_states: bool,
    apply_speculative_contact_restitution: bool,
    // btAlignedObjectArray<btActionInterface*> m_actions;
    actions: Vec<()>,
    // int m_profileTimings;
    latency_motion_state_interpolation: bool,
    // predictive_manifolds: Vec<PersistentManifold>,
    // btSpinMutex m_predictiveManifoldsMutex;
}

impl DiscreteDynamicsWorld {
    pub fn new(
        dispatcher: CollisionDispatcher,
        pair_cache: Box<dyn BroadphaseInterface>,
        constraint_solver: SequentialImpulseConstraintSolver,
    ) -> Self {
        Self {
            dynamics_world: DynamicsWorld::new(dispatcher, pair_cache),
            // sorted_constraints: Vec::new(),
            solver_island_callback: InplaceSolverIslandCallback::new(constraint_solver),
            gravity: Vec3A::new(0.0, -10.0, 0.0),
            local_time: 0.0,
            fixed_time_step: 0.0,
            synchronize_all_motion_states: false,
            apply_speculative_contact_restitution: false,
            latency_motion_state_interpolation: true,
            // island_manager: SimulationIslandManager::default(),
            // constraints: Vec::new(),
            non_static_rigid_bodies: Vec::new(),
            // predictive_manifolds: Vec::new(),
            actions: Vec::new(),
        }
    }

    pub fn set_gravity(&mut self, gravity: Vec3A) {
        self.gravity = gravity;
    }

    fn add_collision_object(&mut self, body: Rc<RefCell<CollisionObject>>, group: i32, mask: i32) {
        self.dynamics_world
            .collision_world
            .add_collision_object(body, group, mask);
    }

    pub fn add_rigid_body_default(&mut self, body: Rc<RefCell<RigidBody>>) {
        if !body
            .borrow()
            .collision_object
            .borrow()
            .is_static_or_kinematic_object()
            && body.borrow().get_flags() & RigidBodyFlags::DisableWorldGravity as i32 == 0
        {
            body.borrow_mut().set_gravity(self.gravity);
        }

        if body
            .borrow()
            .collision_object
            .borrow()
            .get_collision_shape()
            .is_some()
        {
            let co = body.borrow().collision_object.clone();
            if !co.borrow().is_static_object() {
                self.non_static_rigid_bodies.push(body);
            } else {
                co.borrow_mut().set_activation_state(ISLAND_SLEEPING);
            }

            let (group, mask) = if co.borrow().is_static_or_kinematic_object() {
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

            self.add_collision_object(co, group, mask);
        }
    }

    pub fn add_rigid_body(&mut self, body: Rc<RefCell<RigidBody>>, group: i32, mask: i32) {
        if !body
            .borrow()
            .collision_object
            .borrow()
            .is_static_or_kinematic_object()
            && body.borrow().get_flags() & RigidBodyFlags::DisableWorldGravity as i32 == 0
        {
            body.borrow_mut().set_gravity(self.gravity);
        }

        if body
            .borrow()
            .collision_object
            .borrow()
            .get_collision_shape()
            .is_some()
        {
            let co = body.borrow().collision_object.clone();
            if !co.borrow().is_static_object() {
                self.non_static_rigid_bodies.push(body);
            } else {
                co.borrow_mut().set_activation_state(ISLAND_SLEEPING);
            }

            self.add_collision_object(co, group, mask);
        }
    }

    fn apply_gravity(&mut self) {
        for body in &self.non_static_rigid_bodies {
            if body.borrow().collision_object.borrow().is_active() {
                body.borrow_mut().apply_gravity();
            }
        }
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        for body in &self.non_static_rigid_bodies {
            debug_assert!(
                !body
                    .borrow()
                    .collision_object
                    .borrow()
                    .is_static_or_kinematic_object()
            );

            let mut body = body.borrow_mut();
            body.apply_damping(time_step);

            let predicted_transform = body.predict_integration_transform(time_step);
            body.collision_object
                .borrow_mut()
                .interpolation_world_transform = predicted_transform;
        }
    }

    fn create_predictive_contacts_internal(&mut self, time_step: f32) {
        for body in &self.non_static_rigid_bodies {
            let body = body.borrow();
            let mut co = body.collision_object.borrow_mut();
            co.hit_fraction = 1.0;

            debug_assert!(!co.is_static_or_kinematic_object());
            if self
                .dynamics_world
                .collision_world
                .dispatcher_info
                .use_continuous
                && co.is_active()
            {
                let predicted_trans = co.interpolation_world_transform;

                let square_motion = (predicted_trans.translation
                    - co.get_world_transform().translation)
                    .length_squared();

                if co.get_ccd_square_motion_threshold() != 0.0
                    && co.get_ccd_square_motion_threshold() < square_motion
                {
                    unimplemented!();
                }
            }
        }
    }

    fn release_predictive_contacts(&mut self) {
        // self.predictive_manifolds.clear();
    }

    fn create_predictive_contacts(&mut self, time_step: f32) {
        self.release_predictive_contacts();

        debug_assert!(!self.non_static_rigid_bodies.is_empty());
        self.create_predictive_contacts_internal(time_step);
    }

    fn calculation_simulation_islands(&mut self) {
        for manifold in &self.dynamics_world.collision_world.dispatcher1.manifolds {
            let body0 = &manifold.body0;
            let body1 = &manifold.body1;

            let mut body0_ref = body0.borrow_mut();
            let mut body1_ref = body1.borrow_mut();

            if body0_ref.get_activation_state() != ISLAND_SLEEPING
                || body1_ref.get_activation_state() != ISLAND_SLEEPING
            {
                if body0_ref.is_kinematic_object()
                    && body0_ref.get_activation_state() != ISLAND_SLEEPING
                {
                    body1_ref.activate();
                }

                if body1_ref.is_kinematic_object()
                    && body1_ref.get_activation_state() != ISLAND_SLEEPING
                {
                    body0_ref.activate();
                }

                debug_assert!(
                    !body0_ref.is_static_or_kinematic_object()
                        || !body1_ref.is_static_or_kinematic_object()
                );
            }
        }
    }

    fn solve_constraints(&mut self) {
        self.solver_island_callback.solver.solve_group(
            &self.non_static_rigid_bodies,
            &mut self.dynamics_world.collision_world.dispatcher1.manifolds,
            &self.dynamics_world.solver_info,
        );
    }

    fn integrate_transforms_internal(&mut self, time_step: f32) {
        for body in &self.non_static_rigid_bodies {
            let mut body = body.borrow_mut();
            let mut co = body.collision_object.borrow_mut();
            co.hit_fraction = 1.0;

            if !co.is_active() || co.is_static_or_kinematic_object() {
                continue;
            }

            drop(co);

            let predicted_trans = body.predict_integration_transform(time_step);
            body.set_center_of_mass_transform(predicted_trans);
        }
    }

    fn integrate_transforms(&mut self, time_step: f32) {
        if !self.non_static_rigid_bodies.is_empty() {
            self.integrate_transforms_internal(time_step);
        }

        debug_assert!(!self.apply_speculative_contact_restitution);
    }

    fn update_actions(&mut self, _time_step: f32) {
        assert!(self.actions.is_empty());
    }

    fn update_activation_state(&mut self, time_step: f32) {
        for body in &self.non_static_rigid_bodies {
            let mut body = body.borrow_mut();

            body.update_deactivation(time_step);

            if body.wants_sleeping() {
                let mut co = body.collision_object.borrow_mut();
                if co.is_static_or_kinematic_object() {
                    co.set_activation_state(ISLAND_SLEEPING);
                } else if co.activation_state_1 == ACTIVE_TAG {
                    co.set_activation_state(WANTS_DEACTIVATION);
                } else if co.activation_state_1 == ISLAND_SLEEPING {
                    drop(co);

                    body.set_angular_velocity(Vec3A::ZERO);
                    body.set_linear_velocity(Vec3A::ZERO);
                }
            } else {
                body.collision_object
                    .borrow_mut()
                    .set_activation_state(ACTIVE_TAG);
            }
        }
    }

    fn clear_forces(&self) {
        for body in &self.non_static_rigid_bodies {
            body.borrow_mut().clear_forces();
        }
    }

    fn internal_single_step_simulation(&mut self, time_step: f32) {
        self.predict_unconstraint_motion(time_step);

        let dispatcher_info = &mut self.dynamics_world.collision_world.dispatcher_info;
        dispatcher_info.time_step = time_step;
        dispatcher_info.step_count = 0;

        // todo: are there never any predictive contacts?
        // (remove if so)
        self.create_predictive_contacts(time_step);

        self.dynamics_world
            .collision_world
            .perform_discrete_collision_detection();

        self.calculation_simulation_islands();

        self.dynamics_world.solver_info.time_step = time_step;

        self.solve_constraints();
        self.integrate_transforms(time_step);
        self.update_actions(time_step);
    }

    pub fn step_simulation(
        &mut self,
        time_step: f32,
        mut max_sub_steps: u32,
        mut fixed_time_step: f32,
    ) {
        let mut num_simulation_sub_steps = 0;

        if max_sub_steps == 0 {
            fixed_time_step = time_step;
            self.local_time = if self.latency_motion_state_interpolation {
                0.0
            } else {
                time_step
            };
            self.fixed_time_step = 0.0;

            if time_step.abs() < f32::EPSILON {
                unimplemented!();
            } else {
                num_simulation_sub_steps = 1;
                max_sub_steps = 1;
            }
        } else {
            unimplemented!()
        }

        if num_simulation_sub_steps == 0 {
            unimplemented!();
        } else {
            let clamped_simulation_steps = if num_simulation_sub_steps > max_sub_steps {
                unimplemented!()
            } else {
                num_simulation_sub_steps
            };

            self.apply_gravity();

            for _ in 0..clamped_simulation_steps {
                self.internal_single_step_simulation(fixed_time_step);
            }
        }

        self.clear_forces();
    }
}
