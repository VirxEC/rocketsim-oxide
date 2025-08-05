use crate::bullet::{
    collision::{
        dispatch::collision_object::{
            CollisionFlags, CollisionObject, CollisionObjectTypes, DISABLE_DEACTIVATION,
            ISLAND_SLEEPING, WANTS_DEACTIVATION,
        },
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::transform_util::{integrate_transform, integrate_transform_no_rot},
};
use glam::{Affine3A, Mat3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

pub enum RigidBodyFlags {
    DisableWorldGravity = 1,
}

pub struct RigidBodyConstructionInfo {
    pub mass: f32,
    pub start_world_transform: Affine3A,
    pub collision_shape: CollisionShapes,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub friction: f32,
    // pub rolling_friction: f32,
    // pub spinning_friction: f32,
    pub restitution: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
    pub additional_damping: bool,
    pub additional_damping_factor: f32,
    pub additional_linear_damping_threshold_sqr: f32,
    pub additional_angular_damping_threshold_sqr: f32,
    pub additional_angular_damping_factor: f32,
}

impl RigidBodyConstructionInfo {
    #[must_use]
    pub const fn new(mass: f32, collision_shape: CollisionShapes) -> Self {
        Self {
            mass,
            collision_shape,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            friction: 0.5,
            // rolling_friction: 0.0,
            // spinning_friction: 0.0,
            restitution: 0.0,
            linear_sleeping_threshold: 0.0,
            angular_sleeping_threshold: 1.0,
            additional_damping: false,
            additional_damping_factor: 0.005,
            additional_linear_damping_threshold_sqr: 0.01,
            additional_angular_damping_threshold_sqr: 0.01,
            additional_angular_damping_factor: 0.01,
            start_world_transform: Affine3A::IDENTITY,
        }
    }
}

pub struct RigidBody {
    pub collision_object: Rc<RefCell<CollisionObject>>,
    pub inv_inertia_tensor_world: Mat3A,
    pub linear_velocity: Vec3A,
    pub angular_velocity: Vec3A,
    pub inverse_mass: f32,
    pub linear_factor: Vec3A,
    pub gravity: Vec3A,
    pub gravity_acceleration: Vec3A,
    pub inv_inertia_local: Vec3A,
    pub total_force: Vec3A,
    pub total_torque: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub additional_damping: bool,
    pub additional_damping_factor: f32,
    pub additional_linear_damping_threshold_sqr: f32,
    pub additional_angular_damping_threshold_sqr: f32,
    pub additional_angular_damping_factor: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
    // pub motion_state: Option<Box<dyn MotionState>>,
    // pub constraint_refs: Vec<TypedConstraint>,
    pub rigidbody_flags: i32,
    pub delta_linear_velocity: Vec3A,
    pub delta_angular_velocity: Vec3A,
    pub angular_factor: Vec3A,
    pub inv_mass: Vec3A,
    pub push_velocity: Vec3A,
    pub turn_velocity: Vec3A,
}

impl RigidBody {
    #[must_use]
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        let mut collision_object = CollisionObject::default();
        collision_object.internal_type = CollisionObjectTypes::RigidBody as i32;
        collision_object.set_world_transform(info.start_world_transform);

        collision_object.interpolation_world_transform = *collision_object.get_world_transform();
        collision_object.friction = info.friction;
        // collision_object.rolling_friction = info.rolling_friction;
        // collision_object.spinning_friction = info.spinning_friction;
        collision_object.restitution = info.restitution;
        collision_object.set_collision_shape(info.collision_shape);

        let inverse_mass = if info.mass == 0.0 {
            collision_object.collision_flags |= CollisionFlags::StaticObject as i32;
            0.0
        } else {
            collision_object.collision_flags &= !(CollisionFlags::StaticObject as i32);
            1.0 / info.mass
        };

        let inv_inertia_local = Vec3A::select(
            info.local_inertia.cmpeq(Vec3A::splat(0.0)),
            Vec3A::ZERO,
            1.0 / info.local_inertia,
        );

        let linear_factor = Vec3A::ONE;
        let inv_inertia_tensor_world = Self::get_inertia_tensor(
            collision_object.get_world_transform().matrix3,
            inv_inertia_local,
        );

        Self {
            collision_object: Rc::new(RefCell::new(collision_object)),
            inv_inertia_tensor_world,
            linear_velocity: Vec3A::ZERO,
            angular_velocity: Vec3A::ZERO,
            inverse_mass,
            linear_factor,
            gravity: Vec3A::ZERO,
            gravity_acceleration: Vec3A::ZERO,
            inv_inertia_local,
            total_force: Vec3A::ZERO,
            total_torque: Vec3A::ZERO,
            linear_damping: info.linear_damping.clamp(0.0, 1.0),
            angular_damping: info.angular_damping.clamp(0.0, 1.0),
            additional_damping: info.additional_damping,
            additional_damping_factor: info.additional_damping_factor,
            additional_linear_damping_threshold_sqr: info.additional_linear_damping_threshold_sqr,
            additional_angular_damping_threshold_sqr: info.additional_angular_damping_threshold_sqr,
            additional_angular_damping_factor: info.additional_angular_damping_factor,
            linear_sleeping_threshold: info.linear_sleeping_threshold,
            angular_sleeping_threshold: info.angular_sleeping_threshold,
            // motion_state: info.motion_state,
            // constraint_refs: Vec::new(),
            rigidbody_flags: 0,
            delta_linear_velocity: Vec3A::ZERO,
            delta_angular_velocity: Vec3A::ZERO,
            angular_factor: Vec3A::ONE,
            inv_mass: inverse_mass * linear_factor,
            push_velocity: Vec3A::ZERO,
            turn_velocity: Vec3A::ZERO,
        }
    }

    #[must_use]
    pub const fn get_flags(&self) -> i32 {
        self.rigidbody_flags
    }

    pub fn set_gravity(&mut self, acceleration: Vec3A) {
        if self.inverse_mass != 0.0 {
            self.gravity = acceleration * (1.0 / self.inverse_mass);
        }

        self.gravity_acceleration = acceleration;
    }

    pub fn set_linear_velocity(&mut self, lin_vel: Vec3A) {
        debug_assert!(!lin_vel.is_nan());

        self.collision_object.borrow_mut().update_revision += 1;
        self.linear_velocity = lin_vel;
    }

    pub fn set_angular_velocity(&mut self, ang_vel: Vec3A) {
        debug_assert!(!ang_vel.is_nan());

        self.collision_object.borrow_mut().update_revision += 1;
        self.angular_velocity = ang_vel;
    }

    fn get_inertia_tensor(world_mat: Mat3A, inv_inertia_local: Vec3A) -> Mat3A {
        let mut scaled_mat = world_mat.transpose();
        scaled_mat.x_axis *= inv_inertia_local;
        scaled_mat.y_axis *= inv_inertia_local;
        scaled_mat.z_axis *= inv_inertia_local;

        world_mat * scaled_mat
    }

    pub fn update_inertia_tensor(&mut self) {
        self.inv_inertia_tensor_world = Self::get_inertia_tensor(
            self.collision_object.borrow().get_world_transform().matrix3,
            self.inv_inertia_local,
        );
    }

    pub fn apply_torque_impulse(&mut self, torque: Vec3A) {
        debug_assert!(!torque.is_nan());
        self.angular_velocity += self.inv_inertia_tensor_world * torque * self.angular_factor;
    }

    pub fn apply_impulse(&mut self, impulse: Vec3A, rel_pos: Vec3A) {
        debug_assert_ne!(self.inverse_mass, 0.0);
        self.apply_central_impulse(impulse);
        self.apply_torque_impulse(rel_pos.cross(impulse * self.linear_factor));
    }

    pub fn apply_torque(&mut self, torque: Vec3A) {
        debug_assert!(!torque.is_nan());
        self.total_torque += torque * self.angular_factor;
    }

    pub fn apply_central_impulse(&mut self, impulse: Vec3A) {
        debug_assert!(!impulse.is_nan());
        self.linear_velocity += impulse * self.linear_factor * self.inverse_mass;
    }

    pub fn apply_central_force(&mut self, force: Vec3A) {
        debug_assert!(!force.is_nan());
        self.total_force += force * self.linear_factor;
    }

    pub fn apply_gravity(&mut self) {
        if self
            .collision_object
            .borrow()
            .is_static_or_kinematic_object()
        {
            return;
        }

        self.apply_central_force(self.gravity);
    }

    pub fn apply_damping(&mut self, time_step: f32) {
        self.linear_velocity *= (1.0 - self.linear_damping).powf(time_step);
        self.angular_velocity *= (1.0 - self.angular_damping).powf(time_step);

        if self.additional_damping {
            unimplemented!()
        }
    }

    #[must_use]
    pub fn predict_integration_transform(&self, time_step: f32) -> Affine3A {
        let co = self.collision_object.borrow();
        let mut trans = *co.get_world_transform();

        if co.no_rot {
            integrate_transform_no_rot(&mut trans, self.linear_velocity, time_step)
        } else {
            integrate_transform(
                &mut trans,
                self.linear_velocity,
                self.angular_velocity,
                time_step,
            )
        }

        trans
    }

    pub fn set_center_of_mass_transform(&mut self, xform: Affine3A) {
        let mut co = self.collision_object.borrow_mut();
        debug_assert!(!co.is_kinematic_object());

        co.interpolation_world_transform = xform;
        co.interpolation_linear_velocity = self.linear_velocity;
        co.interpolation_angular_velocity = self.angular_velocity;
        co.set_world_transform(xform);

        drop(co);
        self.update_inertia_tensor();
    }

    #[must_use]
    pub fn get_velocity_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.linear_velocity + self.angular_velocity.cross(rel_pos)
    }

    pub fn update_deactivation(&mut self, time_step: f32) {
        let activation_state = self.collision_object.borrow().activation_state_1;

        if activation_state == ISLAND_SLEEPING || activation_state == DISABLE_DEACTIVATION {
            return;
        }

        let mut co = self.collision_object.borrow_mut();
        if self.linear_velocity.length_squared()
            < self.linear_sleeping_threshold * self.linear_sleeping_threshold
            && self.angular_velocity.length_squared()
                < self.angular_sleeping_threshold * self.angular_sleeping_threshold
        {
            co.deactivation_time += time_step;
        } else {
            co.deactivation_time = 0.0;
            co.set_activation_state(0);
        }
    }

    #[must_use]
    pub fn wants_sleeping(&self) -> bool {
        let co = self.collision_object.borrow();
        let activation_state = co.activation_state_1;

        activation_state == DISABLE_DEACTIVATION
            && (activation_state == ISLAND_SLEEPING
                || activation_state == WANTS_DEACTIVATION
                || co.deactivation_time > 2.0)
    }

    pub const fn clear_forces(&mut self) {
        self.total_force = Vec3A::ZERO;
        self.total_torque = Vec3A::ZERO;
    }

    pub fn get_mass(&self) -> f32 {
        if self.inverse_mass == 0.0 {
            0.0
        } else {
            1.0 / self.inverse_mass
        }
    }
}
