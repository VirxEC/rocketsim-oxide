use glam::Vec3A;
use std::f32::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, PI};

pub const GRAVITY_Z: f32 = -650.;
pub const ARENA_EXTENT_X: f32 = 4096.;
/// Does not include inner-goal
pub const ARENA_EXTENT_Y: f32 = 5120.;
pub const ARENA_HEIGHT: f32 = 2048.;
pub const ARENA_EXTENT_X_HOOPS: f32 = 8900. / 3.;
pub const ARENA_EXTENT_Y_HOOPS: f32 = 3581.;
pub const ARENA_HEIGHT_HOOPS: f32 = 1820.;
pub const ARENA_HEIGHT_DROPSHOT: f32 = 2024.;
pub const FLOOR_HEIGHT_DROPSHOT: f32 = 1.5;
pub const ARENA_COLLISION_BASE_FRICTION: f32 = 0.6;
pub const ARENA_COLLISION_BASE_RESTITUTION: f32 = 0.3;
pub const CAR_MASS_BT: f32 = 180.;
/// Ref: <https://www.reddit.com/r/RocketLeague/comments/bmje9l/comment/emxkwrl/?context=3>
pub const BALL_MASS_BT: f32 = CAR_MASS_BT / 6.;
pub const CAR_COLLISION_FRICTION: f32 = 0.3;
pub const CAR_COLLISION_RESTITUTION: f32 = 0.1;
pub const CARBALL_COLLISION_FRICTION: f32 = 2.0;
pub const CARBALL_COLLISION_RESTITUTION: f32 = 0.0;
pub const CARWORLD_COLLISION_FRICTION: f32 = 0.3;
pub const CARWORLD_COLLISION_RESTITUTION: f32 = 0.3;
pub const CARCAR_COLLISION_FRICTION: f32 = 0.09;
pub const CARCAR_COLLISION_RESTITUTION: f32 = 0.1;
/// Greater than ball radius because of arena mesh collision margin
pub const BALL_REST_Z: f32 = 93.15;
/// Ball can never exceed this angular velocity (radians/s)
pub const BALL_MAX_ANG_SPEED: f32 = 6.;
/// Net-velocity drag multiplier
pub const BALL_DRAG: f32 = 0.03;
pub const BALL_FRICTION: f32 = 0.35;
/// Bounce factor
pub const BALL_RESTITUTION: f32 = 0.6;
/// Z impulse applied to hoops ball on kickoff
pub const BALL_HOOPS_LAUNCH_Z_VEL: f32 = 1000.;
pub const BALL_HOOPS_LAUNCH_DELAY: f32 = 0.265;
pub const CAR_MAX_SPEED: f32 = 2300.;
pub const BALL_MAX_SPEED: f32 = 6000.;
pub const BOOST_MAX: f32 = 100.;
pub const BOOST_USED_PER_SECOND: f32 = BOOST_MAX / 3.;
/// Minimum time we can be boosting for
pub const BOOST_MIN_TIME: f32 = 0.1;
/// uu/s for vel (on the ground)
pub const BOOST_ACCEL_GROUND: f32 = 2975. / 3.;
/// uu/s for vel (airborne)
pub const BOOST_ACCEL_AIR: f32 = 3175. / 3.;
pub const BOOST_SPAWN_AMOUNT: f32 = BOOST_MAX / 3.;
/// Amount of boost recharged per second when recharging
pub const RECHARGE_BOOST_PER_SECOND: f32 = 10.;
/// Delay after the car stops boosting
pub const RECHARGE_BOOST_DELAY: f32 = 0.25;
/// Car can never exceed this angular velocity (radians/s)
pub const CAR_MAX_ANG_SPEED: f32 = 5.5;
pub const SUPERSONIC_START_SPEED: f32 = 2200.;
pub const SUPERSONIC_MAINTAIN_MIN_SPEED: f32 = SUPERSONIC_START_SPEED - 100.;
pub const SUPERSONIC_MAINTAIN_MAX_TIME: f32 = 1.;
pub const POWERSLIDE_RISE_RATE: f32 = 5.;
pub const POWERSLIDE_FALL_RATE: f32 = 2.;
pub const THROTTLE_TORQUE_AMOUNT: f32 = CAR_MASS_BT * 400.;
pub const BRAKE_TORQUE_AMOUNT: f32 = CAR_MASS_BT * (14.25 + (1. / 3.));
/// If we are costing with less than this forward vel, we full-brake
pub const STOPPING_FORWARD_VEL: f32 = 25.;
/// How much the brake is applied when costing
pub const COASTING_BRAKE_FACTOR: f32 = 0.15;
/// If we are braking and moving faster than this, disable throttle
pub const BRAKING_NO_THROTTLE_SPEED_THRESH: f32 = 0.01;
/// Throttle input of less than this is ignored
pub const THROTTLE_DEADZONE: f32 = 0.001;
pub const THROTTLE_AIR_ACCEL: f32 = 200. / 3.;
pub const JUMP_ACCEL: f32 = 4375. / 3.;
pub const JUMP_IMMEDIATE_FORCE: f32 = 875. / 3.;
pub const JUMP_MIN_TIME: f32 = 0.025;
pub const JUMP_RESET_TIME_PAD: f32 = 1. / 40.;
pub const JUMP_MAX_TIME: f32 = 0.2;
/// Can be at most 1.25 seconds after the jump is finished
pub const DOUBLEJUMP_MAX_DELAY: f32 = 1.25;
pub const FLIP_Z_DAMP_120: f32 = 0.35;
pub const FLIP_Z_DAMP_START: f32 = 0.15;
pub const FLIP_Z_DAMP_END: f32 = 0.21;
pub const FLIP_TORQUE_TIME: f32 = 0.65;
pub const FLIP_TORQUE_MIN_TIME: f32 = 0.41;
pub const FLIP_PITCHLOCK_TIME: f32 = 1.;
pub const FLIP_PITCHLOCK_EXTRA_TIME: f32 = 0.3;
pub const FLIP_INITIAL_VEL_SCALE: f32 = 500.;
/// Left/Right
pub const FLIP_TORQUE_X: f32 = 260.;
/// Forward/backward
pub const FLIP_TORQUE_Y: f32 = 224.;
pub const FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE: f32 = 1.;
pub const FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE: f32 = 1.9;
pub const FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE: f32 = 2.5;
pub const FLIP_BACKWARD_IMPULSE_SCALE_X: f32 = 16. / 15.;
pub const BALL_COLLISION_RADIUS_SOCCAR: f32 = 91.25;
pub const BALL_COLLISION_RADIUS_HOOPS: f32 = 96.3831;
pub const BALL_COLLISION_RADIUS_DROPSHOT: f32 = 100.2565;
pub const SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y: f32 = 5124.25;
pub const HOOPS_GOAL_SCORE_THRESHOLD_Z: f32 = 270.;
pub const CAR_TORQUE_SCALE: f32 = 2. * PI / (1 << 16) as f32 * 1000.;
pub const CAR_AUTOFLIP_IMPULSE: f32 = 200.;
pub const CAR_AUTOFLIP_TORQUE: f32 = 50.;
pub const CAR_AUTOFLIP_TIME: f32 = 0.4;
pub const CAR_AUTOFLIP_NORMZ_THRESH: f32 = FRAC_1_SQRT_2;
pub const CAR_AUTOFLIP_ROLL_THRESH: f32 = 2.8;
pub const CAR_AUTOROLL_FORCE: f32 = 100.;
pub const CAR_AUTOROLL_TORQUE: f32 = 80.;
pub const BALL_CAR_EXTRA_IMPULSE_Z_SCALE: f32 = 0.35;
pub const BALL_CAR_EXTRA_IMPULSE_Z_SCALE_HOOPS_GROUND: f32 = BALL_CAR_EXTRA_IMPULSE_Z_SCALE * 1.55;
pub const BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE: f32 = 0.65;
pub const BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL_UU: f32 = 4600.;
pub const BALL_CAR_EXTRA_IMPULSE_Z_SCALE_HOOPS_NORMAL_Z_THRESH: f32 = 0.1;
pub const CAR_SPAWN_REST_Z: f32 = 17.;
pub const CAR_RESPAWN_Z: f32 = 36.;
pub const BUMP_COOLDOWN_TIME: f32 = 0.25;
pub const BUMP_MIN_FORWARD_DIST: f32 = 64.5;
pub const DEMO_RESPAWN_TIME: f32 = 3.;
pub const CAR_AIR_CONTROL_TORQUE: Vec3A = Vec3A::new(130., 95., 400.);
pub const CAR_AIR_CONTROL_DAMPING: Vec3A = Vec3A::new(30., 20., 50.);
pub const CAR_SPAWN_LOCATION_AMOUNT: i32 = 5;
pub const CAR_SPAWN_LOCATION_AMOUNT_HEATSEEKER: i32 = 4;
pub const CAR_RESPAWN_LOCATION_AMOUNT: i32 = 4;

pub struct CarSpawnPos {
    pub x: f32,
    pub y: f32,
    pub yaw_ang: f32,
}

impl CarSpawnPos {
    #[inline]
    #[must_use]
    pub const fn new(x: f32, y: f32, yaw_ang: f32) -> Self {
        Self { x, y, yaw_ang }
    }
}

pub const CAR_SPAWN_LOCATIONS_SOCCAR: [CarSpawnPos; CAR_SPAWN_LOCATION_AMOUNT as usize] = [
    CarSpawnPos::new(-2560., -2560., FRAC_PI_4 * 1.),
    CarSpawnPos::new(-2560., -2560., FRAC_PI_4 * 3.),
    CarSpawnPos::new(-3840., -3840., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-3840., -3840., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-4608., -4608., FRAC_PI_4 * 2.),
];
pub const CAR_SPAWN_LOCATIONS_HOOPS: [CarSpawnPos; CAR_SPAWN_LOCATION_AMOUNT as usize] = [
    CarSpawnPos::new(-3072., -3072., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-3072., -3072., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-2816., -2816., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-2816., -2816., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-3200., -3200., FRAC_PI_4 * 2.),
];
pub const CAR_SPAWN_LOCATIONS_DROPSHOT: [CarSpawnPos; CAR_SPAWN_LOCATION_AMOUNT as usize] = [
    CarSpawnPos::new(-2380., -2380., FRAC_PI_4 * 1.),
    CarSpawnPos::new(-2380., -2380., FRAC_PI_4 * 3.),
    CarSpawnPos::new(-3576., -3576., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-3576., -3576., FRAC_PI_4 * 2.),
    CarSpawnPos::new(-4088., -4088., FRAC_PI_4 * 2.),
];
pub const CAR_SPAWN_LOCATIONS_HEATSEEKER: [CarSpawnPos;
    CAR_SPAWN_LOCATION_AMOUNT_HEATSEEKER as usize] = [
    CarSpawnPos::new(-4620., -4620., FRAC_PI_2),
    CarSpawnPos::new(-4620., -4620., FRAC_PI_2),
    CarSpawnPos::new(-4620., -4620., FRAC_PI_2),
    CarSpawnPos::new(-4620., -4620., FRAC_PI_2),
];
pub const CAR_RESPAWN_LOCATIONS_SOCCAR: [CarSpawnPos; CAR_RESPAWN_LOCATION_AMOUNT as usize] = [
    CarSpawnPos::new(-4608., -4608., FRAC_PI_2),
    CarSpawnPos::new(-4608., -4608., FRAC_PI_2),
    CarSpawnPos::new(-4608., -4608., FRAC_PI_2),
    CarSpawnPos::new(-4608., -4608., FRAC_PI_2),
];
pub const CAR_RESPAWN_LOCATIONS_HOOPS: [CarSpawnPos; CAR_RESPAWN_LOCATION_AMOUNT as usize] = [
    CarSpawnPos::new(-3072., -3072., FRAC_PI_2),
    CarSpawnPos::new(-3072., -3072., FRAC_PI_2),
    CarSpawnPos::new(-3072., -3072., FRAC_PI_2),
    CarSpawnPos::new(-3072., -3072., FRAC_PI_2),
];
pub const CAR_RESPAWN_LOCATIONS_DROPSHOT: [CarSpawnPos; CAR_RESPAWN_LOCATION_AMOUNT as usize] = [
    CarSpawnPos::new(-3410., -3410., FRAC_PI_2),
    CarSpawnPos::new(-3100., -3100., FRAC_PI_2),
    CarSpawnPos::new(-3410., -3410., FRAC_PI_2),
    CarSpawnPos::new(-3100., -3100., FRAC_PI_2),
];

pub struct LinearPieceCurve<const N: usize> {
    pub value_mappings: [(f32, f32); N],
}

impl<const N: usize> LinearPieceCurve<N> {
    /// Returns the output of the curve
    ///
    /// # Arguments
    ///
    /// * `input` - The input to the curve
    /// * `default_output` - The default output if N is 0
    #[must_use]
    pub fn get_output(&self, input: f32, default_output: Option<f32>) -> f32 {
        if N == 0 {
            return default_output.unwrap_or(1.);
        }

        let first_val_pair = self.value_mappings[0];

        if input <= first_val_pair.0 {
            return first_val_pair.1;
        }

        for i in 1..N {
            let after_pair = self.value_mappings[i];
            let before_pair = self.value_mappings[i - 1];

            if after_pair.0 > input {
                let range_between = after_pair.0 - before_pair.0;
                let val_diff_between = after_pair.1 - before_pair.1;
                let linear_interp_factor = (input - before_pair.0) / range_between;
                return val_diff_between.mul_add(linear_interp_factor, before_pair.1);
            }
        }

        self.value_mappings[N - 1].1
    }
}

pub const STEER_ANGLE_FROM_SPEED_CURVE: LinearPieceCurve<6> = LinearPieceCurve {
    value_mappings: [
        (0., 0.53356),
        (500., 0.31930),
        (1000., 0.18203),
        (1500., 0.10570),
        (1750., 0.08507),
        (3000., 0.03454),
    ],
};
pub const STEER_ANGLE_FROM_SPEED_CURVE_THREEWHEEL: LinearPieceCurve<2> = LinearPieceCurve {
    value_mappings: [(0., 0.342473), (2300., 0.034837)],
};
pub const POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE: LinearPieceCurve<2> = LinearPieceCurve {
    value_mappings: [(0., 0.39235), (2500., 0.12610)],
};
pub const DRIVE_SPEED_TORQUE_FACTOR_CURVE: LinearPieceCurve<3> = LinearPieceCurve {
    value_mappings: [(0., 1.0), (1400., 0.1), (1410., 0.0)],
};
pub const NON_STICKY_FRICTION_FACTOR_CURVE: LinearPieceCurve<3> = LinearPieceCurve {
    value_mappings: [(0., 0.1), (0.7075, 0.5), (1., 1.0)],
};
pub const LAT_FRICTION_CURVE: LinearPieceCurve<2> = LinearPieceCurve {
    value_mappings: [(0., 1.0), (1., 0.2)],
};
pub const LAT_FRICTION_CURVE_THREEWHEEL: LinearPieceCurve<2> = LinearPieceCurve {
    value_mappings: [(0., 0.30), (1., 0.25)],
};
pub const LONG_FRICTION_CURVE: LinearPieceCurve<0> = LinearPieceCurve { value_mappings: [] };
pub const HANDBRAKE_LAT_FRICTION_FACTOR_CURVE: LinearPieceCurve<1> = LinearPieceCurve {
    value_mappings: [(0., 0.1)],
};
pub const HANDBRAKE_LONG_FRICTION_FACTOR_CURVE: LinearPieceCurve<2> = LinearPieceCurve {
    value_mappings: [(0., 0.5), (1., 0.9)],
};
pub const BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE: LinearPieceCurve<4> = LinearPieceCurve {
    value_mappings: [(0., 0.65), (500., 0.65), (2300., 0.55), (4600., 0.30)],
};
pub const BUMP_VEL_AMOUNT_GROUND_CURVE: LinearPieceCurve<3> = LinearPieceCurve {
    value_mappings: [(0., (5. / 6.)), (1400., 1100.), (2200., 1530.)],
};
pub const BUMP_VEL_AMOUNT_AIR_CURVE: LinearPieceCurve<3> = LinearPieceCurve {
    value_mappings: [(0., (5. / 6.)), (1400., 1390.), (2200., 1945.)],
};
pub const BUMP_UPWARD_VEL_AMOUNT_CURVE: LinearPieceCurve<3> = LinearPieceCurve {
    value_mappings: [(0., (2. / 6.)), (1400., 278.), (2200., 417.)],
};

pub mod btvehicle {
    pub const SUSPENSION_FORCE_SCALE_FRONT: f32 = 36. - (1. / 4.);
    pub const SUSPENSION_FORCE_SCALE_BACK: f32 = 54. + (1. / 4.) + (1.5 / 100.);
    pub const SUSPENSION_STIFFNESS: f32 = 500.;
    pub const WHEELS_DAMPING_COMPRESSION: f32 = 25.;
    pub const WHEELS_DAMPING_RELAXATION: f32 = 40.;
    /// TODO: Are we sure this is the same for all cars?
    pub const MAX_SUSPENSION_TRAVEL: f32 = 12.;
    pub const SUSPENSION_SUBTRACTION: f32 = 0.05;
}

pub mod heatseeker {
    use glam::Vec3A;
    use std::f32::consts::PI;

    /// Initial target speed from kickoff (goes to 2985 after the first touch)
    pub const INITIAL_TARGET_SPEED: f32 = 2900.;
    /// Increase of target speed each touch
    pub const TARGET_SPEED_INCREMENT: f32 = 85.;
    /// Minimum time between touches to speed up
    pub const MIN_SPEEDUP_INTERVAL: f32 = 1.;
    /// Y of target point in goal
    pub const TARGET_Y: f32 = 5120.;
    /// Height of target point in goal
    pub const TARGET_Z: f32 = 320.;
    /// Interpolation of horizontal (X+Y) turning
    pub const HORIZONTAL_BLEND: f32 = 1.45;
    /// Interpolation of vertical (Z) turning
    pub const VERTICAL_BLEND: f32 = 0.78;
    /// Interpolation of acceleration towards target speed
    pub const SPEED_BLEND: f32 = 0.3;
    /// Maximum pitch angle of turning
    pub const MAX_TURN_PITCH: f32 = 7000. * PI / (1 << 15) as f32;
    /// Maximum speed the ball can seek at (different from `BALL_MAX_SPEED`)
    pub const MAX_SPEED: f32 = 4600.;
    /// Threshold of wall collision Y backwall distance to change goal targets
    pub const WALL_BOUNCE_CHANGE_Y_THRESH: f32 = 300.;
    /// Threshold of Y normal to trigger bounce-back
    pub const WALL_BOUNCE_CHANGE_Y_NORMAL: f32 = 0.5;
    /// Scale of the extra wall bounce impulse
    pub const WALL_BOUNCE_FORCE_SCALE: f32 = 1. / 3.;
    /// Fraction of upward bounce impulse that goes straight up
    pub const WALL_BOUNCE_UP_FRAC: f32 = 0.3;
    pub const BALL_START_POS: Vec3A = Vec3A::new(-1000., -2220., 92.75);
    pub const BALL_START_VEL: Vec3A = Vec3A::new(0., -65., 650.);
}

pub mod snowday {
    /// Real puck radius varies a bit from point to point but it shouldn't matter
    pub const PUCK_RADIUS: f32 = 114.25;
    pub const PUCK_HEIGHT: f32 = 62.5;
    /// Number of points on each circle of the cylinder
    pub const PUCK_CIRCLE_POINT_AMOUNT: f32 = 20.;
    pub const PUCK_MASS_BT: f32 = 50.;
    pub const PUCK_GROUND_STICK_FORCE: f32 = 70.;
    pub const PUCK_FRICTION: f32 = 0.1;
    pub const PUCK_RESTITUTION: f32 = 0.3;
}

pub mod dropshot {
    use glam::Vec3A;

    const BT_TO_UU: f32 = 50.0;

    pub const BALL_LAUNCH_Z_VEL: f32 = 985.;
    pub const BALL_LAUNCH_DELAY: f32 = 0.26;
    /// Minimum downward speed to damage tiles
    pub const MIN_DOWNWARD_SPEED_TO_DAMAGE: f32 = 250.;
    /// Minimum car->ball delta speed required to accumulate absorbed force
    pub const MIN_CHARGE_HIT_SPEED: f32 = 500.;
    /// Minimum absorbed force to charge/"break open" the ball
    pub const MIN_ABSORBED_FORCE_FOR_CHARGE: f32 = 2500.;
    /// Minimum absorbed force to super-charge the ball
    pub const MIN_ABSORBED_FORCE_FOR_SUPERCHARGE: f32 = 11000.;
    /// Minimum time between damaging tiles
    pub const MIN_DAMAGE_INTERVAL: f32 = 0.1;
    pub const TILE_WIDTH_X: f32 = TILE_HEXAGON_VERTS_BT[1].to_array()[0] * 2. * BT_TO_UU;
    pub const ROW_OFFSET_Y: f32 = (TILE_HEXAGON_VERTS_BT[3].to_array()[1]
        + TILE_HEXAGON_VERTS_BT[4].to_array()[1])
        * BT_TO_UU;
    pub const TILE_OFFSET_Y: f32 = 2.54736 * BT_TO_UU;
    pub const NUM_TILES_PER_TEAM: i32 = 70;
    pub const TEAM_AMOUNT: i32 = 2;
    /// Number decends each row
    pub const TILES_IN_FIRST_ROW: i32 = 13;
    pub const TILES_IN_LAST_ROW: i32 = 7;
    pub const NUM_TILE_ROWS: i32 = TILES_IN_FIRST_ROW - TILES_IN_LAST_ROW + 1;
    pub const TILE_HEXAGON_AABB_MAX: Vec3A = Vec3A::new(7.6643, 8.85, 0.);
    pub const TILE_HEXAGON_VERTS_BT: [Vec3A; 6] = [
        Vec3A::new(0.0, -8.85, 0.),
        Vec3A::new(7.6643, -4.425, 0.),
        Vec3A::new(7.6643, 4.425, 0.),
        Vec3A::new(0.0, 8.85, 0.),
        Vec3A::new(-7.6643, 4.425, 0.),
        Vec3A::new(-7.6643, -4.425, 0.),
    ];
}

pub mod boostpads {
    use glam::Vec3A;

    pub const CYL_HEIGHT: f32 = 95.;
    pub const CYL_RAD_BIG: f32 = 208.;
    pub const CYL_RAD_SMALL: f32 = 144.;
    pub const BOX_HEIGHT: f32 = 64.;
    pub const BOX_RAD_BIG: f32 = 160.;
    pub const BOX_RAD_SMALL: f32 = 120.;
    pub const COOLDOWN_BIG: f32 = 10.;
    pub const COOLDOWN_SMALL: f32 = 4.;
    pub const BOOST_AMOUNT_BIG: f32 = 100.;
    pub const BOOST_AMOUNT_SMALL: f32 = 12.;
    pub const LOCS_AMOUNT_SMALL_SOCCAR: usize = 28;
    pub const LOCS_AMOUNT_SMALL_HOOPS: usize = 14;
    pub const LOCS_AMOUNT_BIG: usize = 6;
    pub const LOCS_SMALL_SOCCAR: [Vec3A; LOCS_AMOUNT_SMALL_SOCCAR] = [
        Vec3A::new(0., -4240., 70.),
        Vec3A::new(-1792., -4184., 70.),
        Vec3A::new(1792., -4184., 70.),
        Vec3A::new(-940., -3308., 70.),
        Vec3A::new(940., -3308., 70.),
        Vec3A::new(0., -2816., 70.),
        Vec3A::new(-3584., -2484., 70.),
        Vec3A::new(3584., -2484., 70.),
        Vec3A::new(-1788., -2300., 70.),
        Vec3A::new(1788., -2300., 70.),
        Vec3A::new(-2048., -1036., 70.),
        Vec3A::new(0., -1024., 70.),
        Vec3A::new(2048., -1036., 70.),
        Vec3A::new(-1024., 0., 70.),
        Vec3A::new(1024., 0., 70.),
        Vec3A::new(-2048., 1036., 70.),
        Vec3A::new(0., 1024., 70.),
        Vec3A::new(2048., 1036., 70.),
        Vec3A::new(-1788., 2300., 70.),
        Vec3A::new(1788., 2300., 70.),
        Vec3A::new(-3584., 2484., 70.),
        Vec3A::new(3584., 2484., 70.),
        Vec3A::new(0., 2816., 70.),
        Vec3A::new(-940., 3308., 70.),
        Vec3A::new(940., 3308., 70.),
        Vec3A::new(-1792., 4184., 70.),
        Vec3A::new(1792., 4184., 70.),
        Vec3A::new(0., 4240., 70.),
    ];
    pub const LOCS_BIG_SOCCAR: [Vec3A; LOCS_AMOUNT_BIG] = [
        Vec3A::new(-3584., 0., 73.),
        Vec3A::new(3584., 0., 73.),
        Vec3A::new(-3072., 4096., 73.),
        Vec3A::new(3072., 4096., 73.),
        Vec3A::new(-3072., -4096., 73.),
        Vec3A::new(3072., -4096., 73.),
    ];
    pub const LOCS_BIG_HOOPS: [Vec3A; LOCS_AMOUNT_BIG] = [
        Vec3A::new(-2176., 2944., 72.),
        Vec3A::new(2176., -2944., 72.),
        Vec3A::new(-2176., -2944., 72.),
        Vec3A::new(-2432., 0., 72.),
        Vec3A::new(2432., 0., 72.),
        Vec3A::new(2175.99, 2944., 72.),
    ];
    pub const LOCS_SMALL_HOOPS: [Vec3A; LOCS_AMOUNT_SMALL_HOOPS] = [
        Vec3A::new(1536., -1024., 64.),
        Vec3A::new(-1280., -2304., 64.),
        Vec3A::new(0., -2816., 64.),
        Vec3A::new(-1536., -1024., 64.),
        Vec3A::new(1280., -2304., 64.),
        Vec3A::new(-512., 512., 64.),
        Vec3A::new(-1536., 1024., 64.),
        Vec3A::new(1536., 1024., 64.),
        Vec3A::new(1280., 2304., 64.),
        Vec3A::new(0., 2816., 64.),
        Vec3A::new(512., 512., 64.),
        Vec3A::new(512., -512., 64.),
        Vec3A::new(-512., -512., 64.),
        Vec3A::new(-1280., 2304., 64.),
    ];
}
