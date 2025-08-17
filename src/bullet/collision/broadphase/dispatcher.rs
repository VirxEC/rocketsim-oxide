pub struct DispatcherInfo {
    pub time_step: f32,
    pub step_count: u32,
    // pub dispatch_func: DispatchFunc,
    // pub time_of_impact: RefCell<f32>,
    pub use_continuous: bool,
    // pub enable_sat_convex: bool,
    // pub enable_spu: bool,
    // pub use_epa: bool,
    // pub allowed_ccd_penetration: f32,
    // pub use_convex_conservative_distance_util: bool,
    // pub convex_conservative_distance_threshold: f32,
    // pub deterministic_overlapping_pairs: bool,
}

impl Default for DispatcherInfo {
    fn default() -> Self {
        Self {
            time_step: 0.0,
            step_count: 0,
            // dispatch_func: DispatchFunc::DispatchDiscrete,
            // time_of_impact: RefCell::new(1.0),
            use_continuous: true,
            // enable_sat_convex: false,
            // enable_spu: true,
            // use_epa: true,
            // allowed_ccd_penetration: 0.04,
            // use_convex_conservative_distance_util: false,
            // convex_conservative_distance_threshold: 0.0,
            // deterministic_overlapping_pairs: false,
        }
    }
}
