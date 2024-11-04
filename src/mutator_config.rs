use crate::{arena::GameMode, bullet::linear_math::vector3::Vector3, consts};

#[derive(Default)]
pub enum DemoMode {
    #[default]
    Normal,
    OnContact,
    Disabled,
}

pub struct MutatorConfig {
    pub gravity: Vector3,
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
    pub boost_accel: f32,
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
    pub demo_mode: DemoMode,
    pub enable_team_demos: bool,
}

impl MutatorConfig {
    pub fn new(game_mode: GameMode) -> Self {
        Self {
            gravity: Vector3::new(0., 0., 650.),
            car_mass: consts::CAR_MASS_BT,
            car_world_friction: consts::CARWORLD_COLLISION_FRICTION,
            car_world_restitution: consts::CARWORLD_COLLISION_RESTITUTION,
            ball_mass: if game_mode == GameMode::SnowDay {
                consts::snowday::PUCK_MASS_BT
            } else {
                consts::BALL_MASS_BT
            },
            ball_max_speed: consts::BALL_MAX_SPEED,
            ball_drag: consts::BALL_DRAG,
            ball_world_friction: if game_mode == GameMode::SnowDay {
                consts::snowday::PUCK_FRICTION
            } else {
                consts::BALL_FRICTION
            },
            ball_world_restitution: if game_mode == GameMode::SnowDay {
                consts::snowday::PUCK_RESTITUTION
            } else {
                consts::BALL_RESTITUTION
            },
            jump_accel: consts::JUMP_ACCEL,
            jump_immediate_force: consts::JUMP_IMMEDIATE_FORCE,
            boost_accel: consts::BOOST_ACCEL,
            boost_used_per_second: consts::BOOST_USED_PER_SECOND,
            respawn_delay: consts::DEMO_RESPAWN_TIME,
            bump_cooldown_time: consts::BUMP_COOLDOWN_TIME,
            boost_pad_cooldown_big: consts::boostpads::COOLDOWN_BIG,
            boost_pad_cooldown_small: consts::boostpads::COOLDOWN_SMALL,
            car_spawn_boost_amount: consts::BOOST_SPAWN_AMOUNT,
            ball_hit_extra_force_scale: 1.,
            bump_force_scale: 1.,
            ball_radius: match game_mode {
                GameMode::Hoops => consts::BALL_COLLISION_RADIUS_HOOPS,
                GameMode::SnowDay => consts::snowday::PUCK_RADIUS,
                _ => consts::BALL_COLLISION_RADIUS_SOCCAR,
            },
            unlimited_flips: false,
            unlimited_double_jumps: false,
            demo_mode: DemoMode::Normal,
            enable_team_demos: false,
        }
    }
}
