use glam::Vec3A;

use super::{collision_margin::CONVEX_DISTANCE_MARGIN, convex_shape::ConvexShape};

pub struct ConvexInternalShape {
    pub convex_shape: ConvexShape,
    pub implicit_shape_dimensions: Vec3A,
    pub collision_margin: f32,
    // pub padding: f32,
}

impl Default for ConvexInternalShape {
    fn default() -> Self {
        Self {
            convex_shape: ConvexShape::default(),
            implicit_shape_dimensions: Vec3A::ZERO,
            collision_margin: CONVEX_DISTANCE_MARGIN,
            // padding: 0.0,
        }
    }
}
