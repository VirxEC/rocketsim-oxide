use glam::Vec3A;

use super::collision_margin::CONVEX_DISTANCE_MARGIN;

pub struct ConvexInternalShape {
    pub implicit_shape_dimensions: Vec3A,
    pub collision_margin: f32,
}

impl Default for ConvexInternalShape {
    fn default() -> Self {
        Self {
            implicit_shape_dimensions: Vec3A::ZERO,
            collision_margin: CONVEX_DISTANCE_MARGIN,
        }
    }
}
