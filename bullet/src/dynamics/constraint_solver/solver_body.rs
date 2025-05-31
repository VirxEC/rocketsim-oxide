use crate::dynamics::rigid_body::RigidBody;
use glam::{Affine3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

pub struct SolverBody {
    pub world_transform: Affine3A,
    pub delta_linear_velocity: Vec3A,
    pub delta_angular_velocity: Vec3A,
    pub angular_factor: Vec3A,
    pub linear_factor: Vec3A,
    pub inv_mass: Vec3A,
    pub push_velocity: Vec3A,
    pub turn_velocity: Vec3A,
    pub linear_velocity: Vec3A,
    pub angular_velocity: Vec3A,
    pub external_force_impulse: Vec3A,
    pub external_torque_impulse: Vec3A,
    pub original_body: Option<Rc<RefCell<RigidBody>>>,
}

impl SolverBody {
    pub const DEFAULT: Self = Self {
        world_transform: Affine3A::IDENTITY,
        delta_linear_velocity: Vec3A::ZERO,
        delta_angular_velocity: Vec3A::ZERO,
        angular_factor: Vec3A::ONE,
        linear_factor: Vec3A::ONE,
        inv_mass: Vec3A::ZERO,
        push_velocity: Vec3A::ZERO,
        turn_velocity: Vec3A::ZERO,
        linear_velocity: Vec3A::ZERO,
        angular_velocity: Vec3A::ZERO,
        external_force_impulse: Vec3A::ZERO,
        external_torque_impulse: Vec3A::ZERO,
        original_body: None,
    };

    pub fn new(rb: Rc<RefCell<RigidBody>>, time_step: f32) -> Self {
        let rb_ref = rb.borrow();

        let world_transform = *rb_ref.collision_object.borrow().get_world_transform();
        Self {
            world_transform,
            delta_linear_velocity: Vec3A::ZERO,
            delta_angular_velocity: Vec3A::ZERO,
            angular_factor: rb_ref.angular_factor,
            linear_factor: rb_ref.linear_factor,
            inv_mass: rb_ref.inv_mass,
            push_velocity: Vec3A::ZERO,
            turn_velocity: Vec3A::ZERO,
            linear_velocity: rb_ref.linear_velocity,
            angular_velocity: rb_ref.angular_velocity,
            external_force_impulse: rb_ref.total_force * rb_ref.inverse_mass * time_step,
            external_torque_impulse: rb_ref.inv_inertia_tensor_world
                * rb_ref.total_torque
                * time_step,
            original_body: {
                drop(rb_ref);
                Some(rb)
            },
        }
    }

    pub fn internal_apply_impulse(
        &mut self,
        linear_component: Vec3A,
        angular_component: Vec3A,
        impulse_magnitude: f32,
    ) {
        debug_assert!(self.original_body.is_some());
        self.delta_linear_velocity += linear_component * impulse_magnitude * self.linear_factor;
        self.delta_angular_velocity += angular_component * impulse_magnitude * self.angular_factor;
    }

    #[must_use]
    pub fn get_velocity_in_local_point_no_delta(&self, rel_pos: Vec3A) -> Vec3A {
        if self.original_body.is_some() {
            self.linear_velocity
                + self.external_force_impulse
                + (self.angular_velocity + self.external_torque_impulse).cross(rel_pos)
        } else {
            Vec3A::ZERO
        }
    }
}
