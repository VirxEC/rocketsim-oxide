use glam::Vec3A;

pub struct VehicleRaycasterResult {
    hit_point_in_world: Vec3A,
    hit_normal_in_world: Vec3A,
    dist_fraction: f32,
}

pub struct VehicleRaycaster {
    filter_mask: i32,
}

impl VehicleRaycaster {
    pub fn new(filter_mask: i32) -> Self {
        Self { filter_mask }
    }
}
