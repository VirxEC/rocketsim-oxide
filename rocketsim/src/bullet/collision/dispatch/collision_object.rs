use glam::{Affine3A, Vec3A};

use crate::{
    bullet::{
        collision::shapes::collision_shape::CollisionShapes,
        dynamics::rigid_body::RigidBodyConstructionInfo,
    },
    sim::UserInfoTypes,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActivationState {
    Active,
    Sleeping,
    DisableSimulation,
}

pub enum CollisionFlags {
    StaticObject = 1,
    NoContactResponse = (1 << 1),
    CustomMaterialCallback = (1 << 2),
}

pub struct CollisionObject {
    world_transform: Affine3A,
    pub interpolation_world_transform: Affine3A,
    pub interpolation_linear_velocity: Vec3A,
    pub interpolation_angular_velocity: Vec3A,
    pub contact_processing_threshold: f32,
    broadphase_handle: Option<usize>,
    collision_shape: CollisionShapes,
    pub collision_flags: u8,
    pub companion_id: Option<usize>,
    /// The index of this object in `CollisionWorld`
    pub world_array_index: usize,
    activation_state: ActivationState,
    pub can_sleep: bool,
    pub deactivation_time: f32,
    pub friction: f32,
    pub restitution: f32,
    pub no_rot: bool,
    pub user_pointer: u64,
    pub user_index: UserInfoTypes,
}

impl From<RigidBodyConstructionInfo> for CollisionObject {
    fn from(value: RigidBodyConstructionInfo) -> Self {
        Self {
            world_transform: value.start_world_transform,
            interpolation_world_transform: value.start_world_transform,
            interpolation_linear_velocity: Vec3A::ZERO,
            interpolation_angular_velocity: Vec3A::ZERO,
            contact_processing_threshold: f32::MAX,
            broadphase_handle: None,
            collision_shape: value.collision_shape,
            collision_flags: if value.mass == 0.0 {
                CollisionFlags::StaticObject as u8
            } else {
                0
            },
            companion_id: None,
            world_array_index: 0,
            activation_state: ActivationState::Active,
            deactivation_time: 0.0,
            friction: value.friction,
            restitution: value.restitution,
            no_rot: false,
            user_pointer: 0,
            user_index: UserInfoTypes::default(),
            can_sleep: value.can_sleep,
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

    #[must_use]
    pub const fn get_collision_shape(&self) -> &CollisionShapes {
        &self.collision_shape
    }

    #[must_use]
    pub const fn is_static_object(&self) -> bool {
        self.collision_flags & CollisionFlags::StaticObject as u8 != 0
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

    #[inline]
    #[must_use]
    pub const fn get_activation_state(&self) -> ActivationState {
        self.activation_state
    }

    pub fn set_activation_state(&mut self, new_state: ActivationState) {
        if self.activation_state != ActivationState::DisableSimulation {
            self.activation_state = new_state;
        }
    }

    pub const fn force_activate(&mut self) {
        self.activation_state = ActivationState::Active;
        self.deactivation_time = 0.0;
    }

    pub const fn set_broadphase_handle(&mut self, handle: usize) {
        self.broadphase_handle = Some(handle);
    }

    #[must_use]
    pub const fn get_broadphase_handle(&self) -> Option<usize> {
        self.broadphase_handle
    }
}
