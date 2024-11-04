use crate::bullet::{
    btcollision::dispatch::collision_object::CollisionObject,
    linear_math::{matrix3x3::Matrix3x3, vector3::Vector3},
};

enum CollisionFlags {
    DynamicObject = 0,
    StaticObject = 1,
    KinematicObject = 2,
    NoContactResponse = 4,
    CustomMaterialCallback = 8,
    CharacterObject = 16,
    DisableVisualizeObject = 32,
    DisableSpuCollisionProcessing = 64,
    HasContactStiffnessDamping = 128,
    HasCustomDebugRenderingColor = 256,
    HasFrictionAnchor = 512,
    HasCollisionSoundTrigger = 1024,
}

pub struct RigidBody {
    pub collision_object: CollisionObject,
    mass: f32,
    inv_mass: Vector3,
    local_inertia: Vector3,
    start_world_transform: Matrix3x3,
    gravity: Vector3,
    gravity_acceleration: Vector3,
    collision_flags: i32,
    inverse_mass: f32,
    inv_inertia_local: Vector3,
    linear_factor: Vector3,
}

impl Default for RigidBody {
    #[inline]
    fn default() -> Self {
        Self {
            collision_object: CollisionObject::default(),
            mass: 0.,
            inv_mass: Vector3::ZERO,
            local_inertia: Vector3::ZERO,
            start_world_transform: Matrix3x3::IDENTITY,
            gravity: Vector3::ZERO,
            gravity_acceleration: Vector3::ZERO,
            collision_flags: CollisionFlags::StaticObject as i32,
            inverse_mass: 0.,
            inv_inertia_local: Vector3::ZERO,
            linear_factor: Vector3::ONE,
        }
    }
}

impl RigidBody {
    // pub fn set_mass_props(&mut self, mass: f32, local_inertia: Vector3) {
    //     self.mass = mass;
    //     self.local_inertia = local_inertia;

    //     if self.mass == 0. {
    //         self.collision_flags |= CollisionFlags::StaticObject as i32;
    //         self.inverse_mass = 0.;
    //     } else {
    //         self.collision_flags &= !(CollisionFlags::StaticObject as i32);
    //         self.inverse_mass = 1. / self.mass;
    //     }

    //     self.gravity = self.gravity_acceleration * self.mass;
    //     self.inv_inertia_local = Vector3::new(
    //         if local_inertia.x != 0. { 1. / local_inertia.x } else { 0. },
    //         if local_inertia.y != 0. { 1. / local_inertia.y } else { 0. },
    //         if local_inertia.z != 0. { 1. / local_inertia.z } else { 0. },
    //     );
    //     self.inv_mass = self.linear_factor * self.inverse_mass;
    // }

    pub fn set_gravity(&mut self, acceleration: Vector3) {
        if self.inverse_mass != 0. {
            self.gravity = acceleration * (1. / self.inverse_mass);
        }
        self.gravity_acceleration = acceleration;
    }

    pub fn is_static_or_kinematic_object(&self) -> bool {
        (self.collision_flags
            & (CollisionFlags::KinematicObject as i32 | CollisionFlags::StaticObject as i32))
            != 0
    }

    pub fn is_static_object(&self) -> bool {
        (self.collision_flags & CollisionFlags::StaticObject as i32) != 0
    }
}
