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
        collision_object::{CollisionObject, ISLAND_SLEEPING},
        simulation_island_manager::{IslandCallback, SimulationIslandManager},
    },
    narrowphase::persistent_manifold::PersistentManifold,
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

struct InplaceSolverIslandCallback {
    solver_info: Option<ContactSolverInfo>,
    solver: SequentialImpulseConstraintSolver,
    // btTypedConstraint **m_sortedConstraints;
    num_constraints: usize,
    bodies: Vec<CollisionObject>,
    manifolds: Vec<PersistentManifold>,
    constraints: Vec<TypedConstraint>,
}

impl InplaceSolverIslandCallback {
    fn new(solver: SequentialImpulseConstraintSolver) -> Self {
        Self {
            solver,
            solver_info: None,
            num_constraints: 0,
            bodies: Vec::new(),
            manifolds: Vec::new(),
            constraints: Vec::new(),
        }
    }
}

impl IslandCallback for InplaceSolverIslandCallback {}

pub struct DiscreteDynamicsWorld {
    pub dynamics_world: DynamicsWorld,
    sorted_constraints: Vec<TypedConstraint>,
    solver_island_callback: InplaceSolverIslandCallback,
    island_manager: SimulationIslandManager,
    constraints: Vec<TypedConstraint>,
    non_static_rigid_bodies: Vec<Rc<RefCell<RigidBody>>>,
    gravity: Vec3A,
    local_time: f32,
    fixed_time_step: f32,
    // bool m_ownsIslandManager;
    // bool m_ownsConstraintSolver;
    synchronize_all_motion_states: bool,
    apply_speculative_contact_restitution: bool,
    // btAlignedObjectArray<btActionInterface*> m_actions;
    // int m_profileTimings;
    latency_motion_state_interpolation: bool,
    predictive_manifolds: Vec<PersistentManifold>,
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
            sorted_constraints: Vec::new(),
            solver_island_callback: InplaceSolverIslandCallback::new(constraint_solver),
            gravity: Vec3A::new(0.0, -10.0, 0.0),
            local_time: 0.0,
            fixed_time_step: 0.0,
            synchronize_all_motion_states: false,
            apply_speculative_contact_restitution: false,
            latency_motion_state_interpolation: true,
            island_manager: SimulationIslandManager::default(),
            constraints: Vec::new(),
            non_static_rigid_bodies: Vec::new(),
            predictive_manifolds: Vec::new(),
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
}
