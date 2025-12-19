use glam::Vec3A;

#[derive(Debug, Default)]
pub struct ManifoldPoint {
    pub local_point_a: Vec3A,
    pub local_point_b: Vec3A,
    pub position_world_on_b: Vec3A,
    pub position_world_on_a: Vec3A,
    pub normal_world_on_b: Vec3A,
    pub distance_1: f32,
    pub combined_friction: f32,
    pub combined_restitution: f32,
    pub index_0: i32,
    pub index_1: i32,
    pub applied_impulse: f32,
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
            combined_restitution: 0.0,
            index_0: -1,
            index_1: -1,
            applied_impulse: 0.0,
            lateral_friction_dir_1: Vec3A::ZERO,
            lateral_friction_dir_2: Vec3A::ZERO,
            is_special: false,
        }
    }
}
