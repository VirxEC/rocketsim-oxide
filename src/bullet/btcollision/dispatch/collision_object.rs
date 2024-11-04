use crate::bullet::{
    btcollision::{broadphase::proxy::BroadphaseProxy, shapes::collision_shape::CollisionShape},
    linear_math::transform::Transform,
};

#[derive(Default, PartialEq, Eq)]
pub enum ActivationState {
    #[default]
    ActiveTag = 1,
    IslandSleeping,
    WantsDeactivation,
    DisableDeactivation,
    DisableSimulation,
    FixedBaseMultiBody,
}

#[derive(Default)]
pub struct CollisionObject {
    world_transform: Transform,
    collision_shape: CollisionShape,
    update_revision: u32,
    activation_state1: ActivationState,
    world_array_index: Option<usize>,
    broadphase_handle: Option<BroadphaseProxy>,
}

impl CollisionObject {
    // pub fn set_world_transform(&mut self, world_transform: Transform) {
    //     self.world_transform = world_transform;
    //     self.update_revision += 1;
    // }

    pub fn get_world_transform(&self) -> Transform {
        self.world_transform
    }

    pub fn set_collision_shape<S: Into<CollisionShape>>(&mut self, collision_shape: S) {
        self.collision_shape = collision_shape.into();
        self.update_revision += 1;
    }

    pub fn get_collision_shape(&self) -> &CollisionShape {
        &self.collision_shape
    }

    pub fn set_activation_state(&mut self, state: ActivationState) {
        if self.activation_state1 != ActivationState::DisableDeactivation
            && state == ActivationState::DisableSimulation
        {
            self.activation_state1 = state;
        }
    }

    pub fn has_collision_shape(&self) -> bool {
        self.collision_shape.is_some()
    }

    pub fn is_active(&self) -> bool {
        self.activation_state1 != ActivationState::FixedBaseMultiBody
            && self.activation_state1 != ActivationState::IslandSleeping
            && self.activation_state1 != ActivationState::DisableSimulation
    }

    pub fn set_world_array_index(&mut self, index: usize) {
        self.world_array_index = Some(index);
    }

    pub fn get_world_array_index(&self) -> Option<usize> {
        self.world_array_index
    }

    pub fn get_broadphase_handle(&self) -> Option<&BroadphaseProxy> {
        self.broadphase_handle.as_ref()
    }

    pub fn set_broadcast_handle(&mut self, handle: BroadphaseProxy) {
        self.broadphase_handle = Some(handle);
    }
}
