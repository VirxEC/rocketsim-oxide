use glam::{Affine3A, Vec3A};

use crate::bullet::collision::shapes::collision_shape::CollisionShapes;
use crate::sim::UserInfoTypes;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActivationState {
    Active,
    Sleeping,
    WantsDeactivation,
    DisableDeactivation,
    DisableSimulation,
}

pub enum CollisionFlags {
    // DynamicObject = 0,
    StaticObject = 1,
    KinematicObject = (1 << 1),
    NoContactResponse = (1 << 2),
    CustomMaterialCallback = (1 << 3),
}

#[derive(PartialEq, Eq)]
pub enum CollisionObjectTypes {
    CollisionObject = 1,
    RigidBody = 2,
}

pub struct SpecialResolveInfo {
    pub num_special_collisions: u16,
    pub total_normal: Vec3A,
    pub total_dist: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl Default for SpecialResolveInfo {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl SpecialResolveInfo {
    pub const DEFAULT: Self = Self {
        num_special_collisions: 0,
        total_normal: Vec3A::ZERO,
        total_dist: 0.0,
        restitution: 0.0,
        friction: 0.0,
    };
}

pub struct CollisionObject {
    world_transform: Affine3A,
    pub interpolation_world_transform: Affine3A,
    pub interpolation_linear_velocity: Vec3A,
    pub interpolation_angular_velocity: Vec3A,
    pub contact_processing_threshold: f32,
    broadphase_handle: Option<usize>,
    collision_shape: Option<CollisionShapes>,
    pub collision_flags: u8,
    pub companion_id: Option<usize>,
    /// The index of this object in `CollisionWorld`
    pub world_array_index: usize,
    pub activation_state: ActivationState,
    pub deactivation_time: f32,
    pub friction: f32,
    pub restitution: f32,
    pub no_rot: bool,
    pub internal_type: CollisionObjectTypes,
    pub user_pointer: u64,
    pub user_index: UserInfoTypes,
    pub hit_fraction: f32,
    pub special_resolve_info: SpecialResolveInfo,
}

impl Default for CollisionObject {
    fn default() -> Self {
        Self {
            world_transform: Affine3A::IDENTITY,
            interpolation_world_transform: Affine3A::IDENTITY,
            interpolation_linear_velocity: Vec3A::ZERO,
            interpolation_angular_velocity: Vec3A::ZERO,
            contact_processing_threshold: f32::MAX,
            broadphase_handle: None,
            collision_shape: None,
            collision_flags: 0,
            companion_id: None,
            world_array_index: 0,
            activation_state: ActivationState::Active,
            deactivation_time: 0.0,
            friction: 0.5,
            restitution: 0.0,
            no_rot: false,
            internal_type: CollisionObjectTypes::CollisionObject,
            user_pointer: 0,
            user_index: UserInfoTypes::default(),
            hit_fraction: 1.0,
            special_resolve_info: SpecialResolveInfo::default(),
        }
    }
}

impl CollisionObject {
    pub const fn set_world_transform(&mut self, world_trans: Affine3A) {
        self.world_transform = world_trans;
    }

    #[must_use]
    pub const fn get_world_transform(&self) -> &Affine3A {
        &self.world_transform
    }

    pub fn set_collision_shape(&mut self, collision_shape: CollisionShapes) {
        self.collision_shape = Some(collision_shape);
    }

    #[must_use]
    pub const fn get_collision_shape(&self) -> Option<&CollisionShapes> {
        self.collision_shape.as_ref()
    }

    #[must_use]
    pub const fn is_static_object(&self) -> bool {
        self.collision_flags & CollisionFlags::StaticObject as u8 != 0
    }

    #[must_use]
    pub const fn is_kinematic_object(&self) -> bool {
        self.collision_flags & CollisionFlags::KinematicObject as u8 != 0
    }

    #[must_use]
    pub const fn is_static_or_kinematic_object(&self) -> bool {
        self.collision_flags
            & (CollisionFlags::KinematicObject as u8 | CollisionFlags::StaticObject as u8)
            != 0
    }

    #[must_use]
    pub const fn is_active(&self) -> bool {
        !matches!(
            self.activation_state,
            ActivationState::Sleeping | ActivationState::DisableSimulation
        )
    }

    #[must_use]
    pub const fn has_contact_response(&self) -> bool {
        self.collision_flags & CollisionFlags::NoContactResponse as u8 == 0
    }

    #[must_use]
    pub const fn get_activation_state(&self) -> ActivationState {
        self.activation_state
    }

    pub const fn set_activation_state(&mut self, new_state: ActivationState) {
        if !matches!(
            self.activation_state,
            ActivationState::DisableDeactivation | ActivationState::DisableSimulation
        ) {
            self.activation_state = new_state;
        }
    }

    pub const fn activate(&mut self) {
        if self.is_static_or_kinematic_object() {
            self.set_activation_state(ActivationState::Active);
        }
    }

    pub const fn set_broadphase_handle(&mut self, handle: usize) {
        self.broadphase_handle = Some(handle);
    }

    #[must_use]
    pub const fn get_broadphase_handle(&self) -> Option<usize> {
        self.broadphase_handle
    }
}
