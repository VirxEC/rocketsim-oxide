use std::f32::consts::PI;

pub const TRI_INFO_V0V1_CONVEX: i32 = 1;
pub const TRI_INFO_V1V2_CONVEX: i32 = 2;
pub const TRI_INFO_V2V0_CONVEX: i32 = 4;

pub const TRI_INFO_V0V1_SWAP_NORMALB: i32 = 8;
pub const TRI_INFO_V1V2_SWAP_NORMALB: i32 = 16;
pub const TRI_INFO_V2V0_SWAP_NORMALB: i32 = 32;

#[derive(Clone, Copy)]
pub struct TriangleInfo {
    pub flags: i32,
    pub edge_v0_v1_angle: f32,
    pub edge_v1_v2_angle: f32,
    pub edge_v2_v0_angle: f32,
}

impl Default for TriangleInfo {
    fn default() -> Self {
        Self {
            flags: 0,
            edge_v0_v1_angle: PI * 2.0,
            edge_v1_v2_angle: PI * 2.0,
            edge_v2_v0_angle: PI * 2.0,
        }
    }
}

#[derive(Clone)]
pub struct TriangleInfoMap {
    pub internal_map: Vec<TriangleInfo>,
    pub convex_epsilon: f32,
    pub planar_epsilon: f32,
    pub equal_vertex_threshold: f32,
    pub edge_distance_threshold: f32,
    pub max_edge_angle_threshold: f32,
    // pub zero_area_threshold: f32,
}

impl Default for TriangleInfoMap {
    fn default() -> Self {
        Self {
            internal_map: Vec::new(),
            convex_epsilon: 0.0,
            planar_epsilon: 0.0001,
            equal_vertex_threshold: 0.0001,
            edge_distance_threshold: 0.1,
            // zero_area_threshold: 0.0001 * 0.0001,
            max_edge_angle_threshold: PI * 2.0,
        }
    }
}
