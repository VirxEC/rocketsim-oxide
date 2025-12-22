use glam::{Vec3A, Vec4};

use crate::bullet::linear_math::{LARGE_FLOAT, aabb_util_2::Aabb};

pub struct RayInfo<'a> {
    pub ray_sources: &'a [Vec3A; 4],
    pub ray_targets: &'a [Vec3A; 4],
    pub lambda_max: Vec4,
    pub aabb: Aabb,
}

impl<'a> RayInfo<'a> {
    pub fn new(ray_sources: &'a [Vec3A; 4], ray_targets: &'a [Vec3A; 4]) -> Self {
        // Union AABB of the 4 segments for quick root rejection.
        let mut aabb = Aabb::new(
            ray_sources[0].min(ray_targets[0]),
            ray_sources[0].max(ray_targets[0]),
        );
        for i in 1..4 {
            aabb += Aabb::new(
                ray_sources[i].min(ray_targets[i]),
                ray_sources[i].max(ray_targets[i]),
            );
        }

        Self {
            ray_sources,
            ray_targets,
            lambda_max: Vec4::ONE,
            aabb,
        }
    }

    pub fn calc_pos_dir(&self) -> ([Vec4; 3], [Vec4; 3]) {
        const LARGE_VEC: Vec4 = Vec4::splat(LARGE_FLOAT);

        let sources = [
            Vec4::from_array([
                self.ray_sources[0].x,
                self.ray_sources[1].x,
                self.ray_sources[2].x,
                self.ray_sources[3].x,
            ]),
            Vec4::from_array([
                self.ray_sources[0].y,
                self.ray_sources[1].y,
                self.ray_sources[2].y,
                self.ray_sources[3].y,
            ]),
            Vec4::from_array([
                self.ray_sources[0].z,
                self.ray_sources[1].z,
                self.ray_sources[2].z,
                self.ray_sources[3].z,
            ]),
        ];

        let targets = [
            Vec4::from_array([
                self.ray_targets[0].x,
                self.ray_targets[1].x,
                self.ray_targets[2].x,
                self.ray_targets[3].x,
            ]),
            Vec4::from_array([
                self.ray_targets[0].y,
                self.ray_targets[1].y,
                self.ray_targets[2].y,
                self.ray_targets[3].y,
            ]),
            Vec4::from_array([
                self.ray_targets[0].z,
                self.ray_targets[1].z,
                self.ray_targets[2].z,
                self.ray_targets[3].z,
            ]),
        ];

        let mut inv_dirs = [
            Vec4::ONE / (targets[0] - sources[0]),
            Vec4::ONE / (targets[1] - sources[1]),
            Vec4::ONE / (targets[2] - sources[2]),
        ];

        for inv_dir in &mut inv_dirs {
            *inv_dir = Vec4::select(inv_dir.is_finite_mask(), *inv_dir, LARGE_VEC);
        }

        (sources, inv_dirs)
    }
}
