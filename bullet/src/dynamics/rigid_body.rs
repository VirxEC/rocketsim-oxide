use super::constraint_solver::typed_constraint::TypedConstraint;
use crate::{
    collision::{
        dispatch::collision_object::{CollisionFlags, CollisionObject, CollisionObjectTypes},
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::motion_state::MotionState,
};
use glam::{Affine3A, Mat3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

pub enum RigidBodyFlags {
    DisableWorldGravity = 1,
    EnableGyroscopicForceExplicit = 2,
    EnableGyroscopicForceImplicitWorld = 4,
    EnableGyroscopicForceImplicitBody = 8,
    // EnableGyropscopicForce = EnableGyroscopicForceImplicitBody,
}

pub struct RigidBodyConstructionInfo {
    pub mass: f32,
    pub motion_state: Option<Box<dyn MotionState>>,
    pub start_world_transform: Affine3A,
    pub collision_shape: CollisionShapes,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub friction: f32,
    pub rolling_friction: f32,
    pub spinning_friction: f32,
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
    pub fn new(
        mass: f32,
        motion_state: Option<Box<dyn MotionState>>,
        collision_shape: CollisionShapes,
    ) -> Self {
        Self {
            mass,
            motion_state,
            collision_shape,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            friction: 0.5,
            rolling_friction: 0.0,
            spinning_friction: 0.0,
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
    pub motion_state: Option<Box<dyn MotionState>>,
    pub constraint_refs: Vec<TypedConstraint>,
    pub rigidbody_flags: i32,
    pub delta_linear_velocity: Vec3A,
    pub delta_angular_velocity: Vec3A,
    pub angular_factor: Vec3A,
    pub inv_mass: Vec3A,
    pub push_velocity: Vec3A,
    pub turn_velocity: Vec3A,
}

impl RigidBody {
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        let mut collision_object = CollisionObject::default();
        collision_object.internal_type = CollisionObjectTypes::RigidBody as i32;
        if let Some(motion_state) = info.motion_state.as_ref() {
            motion_state.get_world_transform(collision_object.get_mut_world_transform());
        } else {
            collision_object.set_world_transform(info.start_world_transform);
        }

        collision_object.interpolation_world_transform = *collision_object.get_world_transform();
        collision_object.friction = info.friction;
        collision_object.rolling_friction = info.rolling_friction;
        collision_object.spinning_friction = info.spinning_friction;
        collision_object.restitution = info.restitution;
        collision_object.set_collision_shape(info.collision_shape);

        let inverse_mass = if info.mass == 0.0 {
            collision_object.collision_flags |= CollisionFlags::StaticObject as i32;
            0.0
        } else {
            collision_object.collision_flags &= !(CollisionFlags::StaticObject as i32);
            1.0 / info.mass
        };

        let inv_inertia_local = Vec3A::new(
            if info.local_inertia.x == 0.0 {
                0.0
            } else {
                1.0 / info.local_inertia.x
            },
            if info.local_inertia.y == 0.0 {
                0.0
            } else {
                1.0 / info.local_inertia.y
            },
            if info.local_inertia.z == 0.0 {
                0.0
            } else {
                1.0 / info.local_inertia.z
            },
        );

        let linear_factor = Vec3A::ONE;
        let world_mat = collision_object.get_world_transform().matrix3;
        // TODO: ensure the below is correct
        let inv_inertia_tensor_world = Mat3A::from_cols(
            world_mat.x_axis * inv_inertia_local,
            world_mat.y_axis * inv_inertia_local,
            world_mat.z_axis * inv_inertia_local,
        ) * world_mat.transpose();

        let rigidbody_flags = RigidBodyFlags::EnableGyroscopicForceImplicitBody as i32;

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
            motion_state: info.motion_state,
            constraint_refs: Vec::new(),
            rigidbody_flags,
            delta_linear_velocity: Vec3A::ZERO,
            delta_angular_velocity: Vec3A::ZERO,
            angular_factor: Vec3A::ONE,
            inv_mass: inverse_mass * linear_factor,
            push_velocity: Vec3A::ZERO,
            turn_velocity: Vec3A::ZERO,
        }
    }

    pub fn get_flags(&self) -> i32 {
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

    pub fn update_inertia_tensor(&mut self) {
        let world_mat = self.collision_object.borrow().get_world_transform().matrix3;
        self.inv_inertia_tensor_world = Mat3A::from_cols(
            world_mat.x_axis * self.inv_inertia_local,
            world_mat.y_axis * self.inv_inertia_local,
            world_mat.z_axis * self.inv_inertia_local,
        ) * world_mat.transpose();
    }
}
