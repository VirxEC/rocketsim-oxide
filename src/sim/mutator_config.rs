use glam::Vec3A;

use crate::{GameMode, consts};

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum DemoMode {
    #[default]
    Normal,
    OnContact,
    Disabled,
}

#[derive(Clone, Copy, Debug)]
pub struct MutatorConfig {
    pub gravity: Vec3A,
    pub car_mass: f32,
    pub car_world_friction: f32,
    pub car_world_restitution: f32,
    pub ball_mass: f32,
    pub ball_max_speed: f32,
    pub ball_drag: f32,
    pub ball_world_friction: f32,
    pub ball_world_restitution: f32,
    pub jump_accel: f32,
    pub jump_immediate_force: f32,
    pub boost_accel_ground: f32,
    pub boost_accel_air: f32,
    pub boost_used_per_second: f32,
    pub respawn_delay: f32,
    pub bump_cooldown_time: f32,
    pub boost_pad_cooldown_big: f32,
    pub boost_pad_cooldown_small: f32,
    pub car_spawn_boost_amount: f32,
    pub ball_hit_extra_force_scale: f32,
    pub bump_force_scale: f32,
    pub ball_radius: f32,
    pub unlimited_flips: bool,
    pub unlimited_double_jumps: bool,
    pub recharge_boost_enabled: bool,
    pub recharge_boost_per_second: f32,
    pub recharge_boost_delay: f32,
    pub demo_mode: DemoMode,
    pub enable_team_demos: bool,
    /// Only used if the game mode has soccar goals (i.e. soccar, heatseeker, snowday)
    pub goal_base_threshold_y: f32,
}

impl Default for MutatorConfig {
    fn default() -> Self {
        const { Self::new(GameMode::Soccar) }
    }
}

impl MutatorConfig {
    #[must_use]
    pub const fn new(game_mode: GameMode) -> Self {
        Self {
            gravity: Vec3A::new(0., 0., consts::GRAVITY_Z),
            car_mass: consts::CAR_MASS_BT,
            car_world_friction: consts::CARWORLD_COLLISION_FRICTION,
            car_world_restitution: consts::CARWORLD_COLLISION_RESTITUTION,
            ball_mass: if matches!(game_mode, GameMode::Snowday) {
                consts::snowday::PUCK_MASS_BT
            } else {
                consts::BALL_MASS_BT
            },
            ball_max_speed: consts::BALL_MAX_SPEED,
            ball_drag: consts::BALL_DRAG,
            ball_world_friction: if matches!(game_mode, GameMode::Snowday) {
                consts::snowday::PUCK_FRICTION
            } else {
                consts::BALL_FRICTION
            },
            ball_world_restitution: if matches!(game_mode, GameMode::Snowday) {
                consts::snowday::PUCK_RESTITUTION
            } else {
                consts::BALL_RESTITUTION
            },
            jump_accel: consts::JUMP_ACCEL,
            jump_immediate_force: consts::JUMP_IMMEDIATE_FORCE,
            boost_accel_ground: consts::BOOST_ACCEL_GROUND,
            boost_accel_air: consts::BOOST_ACCEL_AIR,
            boost_used_per_second: consts::BOOST_USED_PER_SECOND,
            respawn_delay: consts::DEMO_RESPAWN_TIME,
            bump_cooldown_time: consts::BUMP_COOLDOWN_TIME,
            boost_pad_cooldown_big: consts::boostpads::COOLDOWN_BIG,
            boost_pad_cooldown_small: consts::boostpads::COOLDOWN_SMALL,
            car_spawn_boost_amount: match game_mode {
                GameMode::Dropshot => 100.,
                _ => consts::BOOST_SPAWN_AMOUNT,
            },
            ball_hit_extra_force_scale: 1.,
            bump_force_scale: 1.,
            ball_radius: match game_mode {
                GameMode::Hoops => consts::BALL_COLLISION_RADIUS_HOOPS,
                GameMode::Snowday => consts::snowday::PUCK_RADIUS,
                GameMode::Dropshot => consts::BALL_COLLISION_RADIUS_DROPSHOT,
                _ => consts::BALL_COLLISION_RADIUS_SOCCAR,
            },
            unlimited_flips: false,
            unlimited_double_jumps: false,
            recharge_boost_enabled: matches!(game_mode, GameMode::Snowday),
            recharge_boost_per_second: consts::RECHARGE_BOOST_PER_SECOND,
            recharge_boost_delay: consts::RECHARGE_BOOST_DELAY,
            demo_mode: DemoMode::Normal,
            enable_team_demos: false,
            goal_base_threshold_y: consts::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y,
        }
    }
}
