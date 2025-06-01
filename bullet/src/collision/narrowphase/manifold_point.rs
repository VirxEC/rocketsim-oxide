use glam::Vec3A;

pub enum ContactPointFlags {
    LateralFrictionInitialized = 1,
    HasContactCfm = 2,
    HasContactErp = 4,
    ContactStiffnessDamping = 8,
    FrictionAnchor = 16,
}

#[derive(Debug, Default)]
pub struct ManifoldPoint {
    pub local_point_a: Vec3A,
    pub local_point_b: Vec3A,
    pub position_world_on_b: Vec3A,
    pub position_world_on_a: Vec3A,
    pub normal_world_on_b: Vec3A,
    pub distance_1: f32,
    pub combined_friction: f32,
    // pub combined_rolling_friction: f32,
    // pub combined_spinning_friction: f32,
    pub combined_restitution: f32,
    pub part_id_0: i32,
    pub part_id_1: i32,
    pub index_0: i32,
    pub index_1: i32,
    // mutable void* m_userPersistentData,
    pub contact_point_flags: i32,
    pub applied_impulse: f32,
    pub prev_rhs: f32,
    pub applied_impulse_lateral_1: f32,
    pub applied_impulse_lateral_2: f32,
    pub contact_motion_1: f32,
    pub contact_motion_2: f32,
    pub contact_cfm: f32,
    // union {
    //     f32 m_contactCFM,
    //     f32 m_combinedContactStiffness1,
    // },
    pub contact_erp: f32,
    // union {
    //     f32 m_contactERP,
    //     f32 m_combinedContactDamping1,
    // },
    pub friction_cfm: f32,
    pub life_time: i32,
    pub lateral_friction_dir_1: Vec3A,
    pub lateral_friction_dir_2: Vec3A,
    pub is_special: bool,
}

impl ManifoldPoint {
    pub const fn new(point_a: Vec3A, point_b: Vec3A, normal: Vec3A, distance: f32) -> Self {
        Self {
            local_point_a: point_a,
            local_point_b: point_b,
            position_world_on_a: Vec3A::ZERO,
            position_world_on_b: Vec3A::ZERO,
            normal_world_on_b: normal,
            distance_1: distance,
            combined_friction: 0.0,
            // combined_rolling_friction: 0.0,
            // combined_spinning_friction: 0.0,
            combined_restitution: 0.0,
            part_id_0: -1,
            part_id_1: -1,
            index_0: -1,
            index_1: -1,
            contact_point_flags: 0,
            applied_impulse: 0.0,
            prev_rhs: 0.0,
            applied_impulse_lateral_1: 0.0,
            applied_impulse_lateral_2: 0.0,
            contact_motion_1: 0.0,
            contact_motion_2: 0.0,
            contact_cfm: 0.0,
            contact_erp: 0.0,
            friction_cfm: 0.0,
            life_time: 0,
            lateral_friction_dir_1: Vec3A::ZERO,
            lateral_friction_dir_2: Vec3A::ZERO,
            is_special: false,
        }
    }
}
