use super::{CollisionMasks, MutatorConfig, PhysState};
use crate::{
    BT_TO_UU, GameMode, UU_TO_BT, UserInfoTypes,
    bullet::{
        collision::{
            broadphase::broadphase_proxy::{BroadphaseNativeTypes, CollisionFilterGroups},
            dispatch::collision_object::{ACTIVE_TAG, CollisionFlags},
            shapes::{collision_shape::CollisionShapes, sphere_shape::SphereShape},
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
    },
    consts,
};
use glam::{Affine3A, Mat3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

#[derive(Clone, Copy, Debug)]
pub struct HeatseekerInfo {
    /// Which net the ball should seek towards;
    /// When 0, no net
    pub y_target_dir: f32,
    pub cur_target_speed: f32,
    pub time_since_hit: f32,
}

impl Default for HeatseekerInfo {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl HeatseekerInfo {
    pub const DEFAULT: Self = Self {
        y_target_dir: 0.,
        cur_target_speed: consts::heatseeker::INITIAL_TARGET_SPEED,
        time_since_hit: 0.,
    };
}

#[derive(Clone, Copy, Debug)]
pub struct DropshotInfo {
    /// Charge level number, which controls the radius of damage when hitting tiles
    /// 1 = damages r=1 -> 1 tile
    /// 2 = damages r=2 -> 7 tiles
    /// 3 = damages r=3 -> 19 tiles
    pub charge_level: i32,
    /// Resets when a tile is damaged
    pub accumulated_hit_force: f32,
    /// Which side of the field the ball can damage (0=none, -1=blue, 1=orange)
    pub y_target_dir: f32,
    pub has_damaged: bool,
    /// Only valid if `has_damaged`
    pub last_damage_tick: u64,
}

impl Default for DropshotInfo {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl DropshotInfo {
    pub const DEFAULT: Self = Self {
        charge_level: 1,
        accumulated_hit_force: 0.,
        y_target_dir: 0.,
        has_damaged: false,
        last_damage_tick: 0,
    };
}

#[derive(Clone, Copy, Debug)]
pub struct BallState {
    pub physics: PhysState,
    pub tick_count_since_update: u64,
    pub hs_info: HeatseekerInfo,
    pub ds_info: DropshotInfo,
}

impl Default for BallState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BallState {
    pub const DEFAULT: Self = Self {
        physics: PhysState {
            pos: Vec3A::new(0.0, 0.0, consts::BALL_REST_Z),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        tick_count_since_update: 0,
        hs_info: HeatseekerInfo::DEFAULT,
        ds_info: DropshotInfo::DEFAULT,
    };
}

pub struct Ball {
    pub(crate) internal_state: BallState,
    pub(crate) rigid_body: Rc<RefCell<RigidBody>>,
    pub(crate) ground_stick_applied: bool,
    pub(crate) velocity_impulse_cache: Vec3A,
}

impl Ball {
    fn make_ball_collision_shape(
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
    ) -> (CollisionShapes, Vec3A) {
        if game_mode == GameMode::Snowday {
            todo!()
        } else {
            let shape = SphereShape::new(mutator_config.ball_radius * UU_TO_BT);
            let local_inertia = shape.calculate_local_inertia(mutator_config.ball_mass);

            (CollisionShapes::Sphere(shape), local_inertia)
        }
    }

    pub(crate) fn new(
        game_mode: GameMode,
        bullet_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        no_rot: bool,
    ) -> Self {
        let (collision_shape, local_inertia) =
            Self::make_ball_collision_shape(game_mode, mutator_config);
        let shape_type = collision_shape.get_shape_type();

        let mut info = RigidBodyConstructionInfo::new(mutator_config.ball_mass, collision_shape);
        info.start_world_transform.translation.z = consts::BALL_REST_Z * UU_TO_BT;
        info.local_inertia = local_inertia;
        info.linear_damping = mutator_config.ball_drag;
        info.friction = mutator_config.ball_world_friction;
        info.restitution = mutator_config.ball_world_restitution;

        let body = RigidBody::new(info);
        let mut co = body.collision_object.borrow_mut();
        co.user_index = UserInfoTypes::Ball;
        co.collision_flags |= CollisionFlags::CustomMaterialCallback as i32;
        co.no_rot = no_rot && shape_type == BroadphaseNativeTypes::SphereShapeProxytype;

        drop(co);

        let rigid_body = Rc::new(RefCell::new(body));

        bullet_world.add_rigid_body(
            rigid_body.clone(),
            CollisionFilterGroups::DefaultFilter as i32
                | CollisionMasks::HoopsNet as i32
                | CollisionMasks::DropshotTile as i32,
            CollisionFilterGroups::AllFilter as i32,
        );

        Self {
            internal_state: BallState::DEFAULT,
            rigid_body,
            ground_stick_applied: false,
            velocity_impulse_cache: Vec3A::ZERO,
        }
    }

    #[must_use]
    pub fn get_state(&self) -> BallState {
        let mut state = self.internal_state;
        let rb = self.rigid_body.borrow();
        state.physics.vel = rb.linear_velocity * BT_TO_UU;
        state.physics.ang_vel = rb.angular_velocity;

        let trans = *rb.collision_object.borrow().get_world_transform();
        state.physics.pos = trans.translation * BT_TO_UU;
        state.physics.rot_mat = trans.matrix3;

        state
    }

    pub fn set_state(&mut self, state: BallState) {
        let mut rb = self.rigid_body.borrow_mut();

        rb.collision_object
            .borrow_mut()
            .set_world_transform(Affine3A {
                matrix3: state.physics.rot_mat,
                translation: state.physics.pos * UU_TO_BT,
            });

        rb.set_linear_velocity(state.physics.vel * UU_TO_BT);
        rb.set_angular_velocity(state.physics.ang_vel);
        rb.update_inertia_tensor();

        if state.physics.vel != Vec3A::ZERO || state.physics.ang_vel != Vec3A::ZERO {
            rb.collision_object
                .borrow_mut()
                .set_activation_state(ACTIVE_TAG);
        }

        self.internal_state = state;
        self.internal_state.tick_count_since_update = 0;
    }

    pub(crate) fn pre_tick_update(&mut self, game_mode: GameMode, _tick_time: f32) {
        match game_mode {
            GameMode::Heatseeker => todo!(),
            GameMode::Snowday => self.ground_stick_applied = false,
            GameMode::Dropshot | GameMode::Hoops => {
                // launch ball after a short delay on kickoff
                todo!()
            }
            _ => {}
        }
    }

    pub(crate) fn finish_physics_tick(&mut self, mutator_config: &MutatorConfig) {
        let mut rb = self.rigid_body.borrow_mut();

        if self.velocity_impulse_cache.length_squared() != 0.0 {
            rb.linear_velocity += self.velocity_impulse_cache;
            self.velocity_impulse_cache = Vec3A::ZERO;
        }

        let ball_max_speed_bt = mutator_config.ball_max_speed * UU_TO_BT;
        if rb.linear_velocity.length_squared() > ball_max_speed_bt * ball_max_speed_bt {
            rb.linear_velocity = rb.linear_velocity.normalize() * ball_max_speed_bt;
        }

        if rb.angular_velocity.length_squared()
            > consts::BALL_MAX_ANG_SPEED * consts::BALL_MAX_ANG_SPEED
        {
            rb.angular_velocity = rb.angular_velocity.normalize() * consts::BALL_MAX_ANG_SPEED;
        }

        self.internal_state.tick_count_since_update += 1;
    }
}
