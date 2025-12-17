use glam::{Affine3A, Vec3A};

use crate::{UserInfoTypes, bullet::collision::shapes::collision_shape::CollisionShapes};

pub const ACTIVE_TAG: i32 = 1;
pub const ISLAND_SLEEPING: i32 = 2;
pub const WANTS_DEACTIVATION: i32 = 3;
pub const DISABLE_DEACTIVATION: i32 = 4;
pub const DISABLE_SIMULATION: i32 = 5;
pub const FIXED_BASE_MULTI_BODY: i32 = 6;

pub enum CollisionFlags {
    // DynamicObject = 0,
    StaticObject = 1,
    KinematicObject = 2,
    NoContactResponse = 4,
    CustomMaterialCallback = 8,
    // CharacterObject = 16,
    // DisableVisualizeObject = 32,
    // DisableSpuCollisionProcessing = 64,
    // HasContactStiffnessDamping = 128,
    // HasCustomDebugRenderingColor = 256,
    // HasFrictionAnchor = 512,
    // HasCollisionSoundTrigger = 1024,
}

pub enum CollisionObjectTypes {
    CollisionObject = 1,
    RigidBody = 2,
    // GhostObject = 4,
    // SoftBody = 8,
    // HfFluid = 16,
    // UserType = 32,
    // FeatherstoneLink = 64,
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
    // pub anisotropic_friction: Vec3A,
    // pub has_anisotropic_friction: bool,
    pub contact_processing_threshold: f32,
    broadphase_handle: Option<usize>,
    collision_shape: Option<CollisionShapes>,
    // void* m_extensionPointer;
    // pub root_collision_shape: Option<Rc<RefCell<CollisionShapes>>>,
    pub collision_flags: i32,
    // pub island_tag_1: i32,
    pub companion_id: Option<usize>,
    world_array_index: usize,
    rigid_body_world_index: usize,
    pub activation_state_1: i32,
    pub deactivation_time: f32,
    pub friction: f32,
    pub restitution: f32,
    // pub contact_damping: f32,
    // pub contact_stiffness: f32,
    pub no_rot: bool,
    pub internal_type: i32,
    // void* m_userObjectPointer;
    pub user_pointer: u64,
    pub user_index: UserInfoTypes,
    // pub user_index_2: i32,
    // pub user_index_3: i32,
    pub hit_fraction: f32,
    // pub ccd_swept_sphere_radius: f32,
    // pub ccd_motion_threshold: f32,
    // pub check_collide_with: bool,
    // btAlignedObjectArray<const btCollisionObject*> m_objectsWithoutCollisionCheck;
    // pub update_revision: u32,
    pub special_resolve_info: SpecialResolveInfo,
}

impl Default for CollisionObject {
    fn default() -> Self {
        Self {
            world_transform: Affine3A::IDENTITY,
            interpolation_world_transform: Affine3A::IDENTITY,
            interpolation_linear_velocity: Vec3A::ZERO,
            interpolation_angular_velocity: Vec3A::ZERO,
            // anisotropic_friction: Vec3A::ONE,
            // has_anisotropic_friction: false,
            contact_processing_threshold: f32::MAX,
            broadphase_handle: None,
            collision_shape: None,
            // root_collision_shape: None,
            collision_flags: 0,
            // island_tag_1: -1,
            companion_id: None,
            world_array_index: 0,
            rigid_body_world_index: 0,
            activation_state_1: ACTIVE_TAG,
            deactivation_time: 0.0,
            friction: 0.5,
            restitution: 0.0,
            // contact_damping: 0.1,
            // contact_stiffness: f32::MAX,
            no_rot: false,
            internal_type: CollisionObjectTypes::CollisionObject as i32,
            user_pointer: 0,
            user_index: UserInfoTypes::default(),
            // user_index_2: -1,
            // user_index_3: -1,
            hit_fraction: 1.0,
            // ccd_swept_sphere_radius: 0.0,
            // ccd_motion_threshold: 0.0,
            // check_collide_with: false,
            // update_revision: 0,
            special_resolve_info: SpecialResolveInfo::default(),
        }
    }
}

impl CollisionObject {
    pub const fn set_world_transform(&mut self, world_trans: Affine3A) {
        // self.update_revision += 1;
        self.world_transform = world_trans;
    }

    #[must_use]
    pub const fn get_world_transform(&self) -> &Affine3A {
        &self.world_transform
    }

    pub fn set_collision_shape(&mut self, collision_shape: CollisionShapes) {
        // self.update_revision += 1;
        self.collision_shape = Some(collision_shape);
        // self.root_collision_shape = Some(collision_shape);
    }

    #[must_use]
    pub const fn get_collision_shape_mut(&mut self) -> Option<&mut CollisionShapes> {
        self.collision_shape.as_mut()
    }

    #[must_use]
    pub const fn get_collision_shape(&self) -> Option<&CollisionShapes> {
        self.collision_shape.as_ref()
    }

    #[must_use]
    pub const fn is_static_object(&self) -> bool {
        self.collision_flags & CollisionFlags::StaticObject as i32 != 0
    }

    #[must_use]
    pub const fn is_kinematic_object(&self) -> bool {
        self.collision_flags & CollisionFlags::KinematicObject as i32 != 0
    }

    #[must_use]
    pub const fn is_static_or_kinematic_object(&self) -> bool {
        self.collision_flags
            & (CollisionFlags::KinematicObject as i32 | CollisionFlags::StaticObject as i32)
            != 0
    }

    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.activation_state_1 != FIXED_BASE_MULTI_BODY
            && self.activation_state_1 != ISLAND_SLEEPING
            && self.activation_state_1 != DISABLE_SIMULATION
    }

    #[must_use]
    pub const fn has_contact_response(&self) -> bool {
        self.collision_flags & CollisionFlags::NoContactResponse as i32 == 0
    }

    #[must_use]
    pub const fn get_activation_state(&self) -> i32 {
        self.activation_state_1
    }

    pub const fn set_activation_state(&mut self, new_state: i32) {
        if self.activation_state_1 != DISABLE_DEACTIVATION
            && self.activation_state_1 != DISABLE_SIMULATION
        {
            self.activation_state_1 = new_state;
        }
    }

    pub const fn activate(&mut self) {
        if self.is_static_or_kinematic_object() {
            self.set_activation_state(ACTIVE_TAG);
        }
    }

    #[must_use]
    pub const fn get_world_array_index(&self) -> usize {
        self.world_array_index
    }

    pub const fn set_world_array_index(&mut self, index: usize) {
        self.world_array_index = index;
    }

    #[must_use]
    pub const fn get_rigid_body_world_index(&self) -> usize {
        self.rigid_body_world_index
    }

    pub const fn set_rigid_body_world_index(&mut self, index: usize) {
        self.rigid_body_world_index = index;
    }

    pub const fn set_broadphase_handle(&mut self, handle: usize) {
        self.broadphase_handle = Some(handle);
    }

    #[must_use]
    pub const fn get_broadphase_handle(&self) -> Option<usize> {
        self.broadphase_handle
    }
}
