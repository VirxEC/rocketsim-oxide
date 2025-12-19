use glam::{Affine3A, Mat3A, Vec3A};

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
    consts::{dropshot, heatseeker},
    sim::{BallHitInfo, Car, Team},
};

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
        cur_target_speed: heatseeker::INITIAL_TARGET_SPEED,
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
    pub(crate) rigid_body_idx: usize,
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

        let mut body = RigidBody::new(info);
        body.collision_object.user_index = UserInfoTypes::Ball;
        body.collision_object.collision_flags |= CollisionFlags::CustomMaterialCallback as u8;
        body.collision_object.no_rot =
            no_rot && shape_type == BroadphaseNativeTypes::SphereShapeProxytype;

        let rigid_body_idx = bullet_world
            .add_rigid_body(
                body,
                CollisionFilterGroups::Default as u8
                    | CollisionMasks::HoopsNet as u8
                    | CollisionMasks::DropshotTile as u8,
                CollisionFilterGroups::All as u8,
            )
            .unwrap();

        Self {
            internal_state: BallState::DEFAULT,
            rigid_body_idx,
            ground_stick_applied: false,
            velocity_impulse_cache: Vec3A::ZERO,
        }
    }

    pub(crate) fn set_state(&mut self, rb: &mut RigidBody, state: BallState) {
        rb.collision_object.set_world_transform(Affine3A {
            matrix3: state.physics.rot_mat,
            translation: state.physics.pos * UU_TO_BT,
        });

        rb.set_linear_velocity(state.physics.vel * UU_TO_BT);
        rb.set_angular_velocity(state.physics.ang_vel);
        rb.update_inertia_tensor();

        if state.physics.vel != Vec3A::ZERO || state.physics.ang_vel != Vec3A::ZERO {
            rb.collision_object.set_activation_state(ACTIVE_TAG);
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

    pub(crate) fn finish_physics_tick(
        &mut self,
        rb: &mut RigidBody,
        mutator_config: &MutatorConfig,
    ) {
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

        self.internal_state.physics.vel = rb.linear_velocity * BT_TO_UU;
        self.internal_state.physics.ang_vel = rb.angular_velocity;

        let trans = *rb.collision_object.get_world_transform();
        self.internal_state.physics.pos = trans.translation * BT_TO_UU;
        self.internal_state.physics.rot_mat = trans.matrix3;

        self.internal_state.tick_count_since_update += 1;
    }

    pub(crate) fn on_hit(
        &mut self,
        car: &mut Car,
        rel_pos: Vec3A,
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
        tick_count: u64,
    ) {
        let mut ball_hit_info = BallHitInfo {
            relative_pos_on_ball: rel_pos,
            tick_count_when_hit: tick_count,
            ball_pos: self.internal_state.physics.pos,
            extra_hit_vel: Vec3A::ZERO,
            tick_count_when_extra_impulse_applied: 0,
        };

        if let Some(old_bhi) = car.internal_state.ball_hit_info {
            ball_hit_info.tick_count_when_extra_impulse_applied =
                old_bhi.tick_count_when_extra_impulse_applied;

            // Once we do an extra car-ball impulse, we need to wait at least 1 tick to do it again
            if tick_count <= old_bhi.tick_count_when_extra_impulse_applied + 1
                && old_bhi.tick_count_when_extra_impulse_applied <= tick_count
            {
                car.internal_state.ball_hit_info = Some(ball_hit_info);
                return;
            }
        }

        ball_hit_info.tick_count_when_extra_impulse_applied = tick_count;

        let car_forward = car.internal_state.physics.rot_mat.x_axis;
        let rel_pos = self.internal_state.physics.pos - car.internal_state.physics.pos;
        let rel_vel = self.internal_state.physics.vel - car.internal_state.physics.vel;

        let rel_speed = rel_vel
            .length()
            .min(consts::BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL_UU);
        if rel_speed > 0.0 {
            let extra_z_scale = game_mode == GameMode::Hoops
                && car.internal_state.is_on_ground
                && car.internal_state.physics.rot_mat.z_axis.z
                    > consts::BALL_CAR_EXTRA_IMPULSE_Z_SCALE_HOOPS_NORMAL_Z_THRESH;
            let z_scale = if extra_z_scale {
                consts::BALL_CAR_EXTRA_IMPULSE_Z_SCALE_HOOPS_GROUND
            } else {
                consts::BALL_CAR_EXTRA_IMPULSE_Z_SCALE
            };

            let mut hit_dir = rel_pos * Vec3A::new(1.0, 1.0, z_scale).normalize();
            let forward_dir_adjustment = car_forward
                * hit_dir.dot(car_forward)
                * const { 1.0 - consts::BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE };
            hit_dir = (hit_dir - forward_dir_adjustment).normalize();

            let added_vel = hit_dir
                * rel_speed
                * consts::BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE.get_output(rel_speed)
                * mutator_config.ball_hit_extra_force_scale;
            ball_hit_info.extra_hit_vel = added_vel;

            self.velocity_impulse_cache += added_vel * UU_TO_BT;
        }

        car.internal_state.ball_hit_info = Some(ball_hit_info);

        match game_mode {
            GameMode::Heatseeker => {
                let can_increase = self.internal_state.hs_info.time_since_hit
                    > heatseeker::MIN_SPEEDUP_INTERVAL
                    || self.internal_state.hs_info.y_target_dir == 0.0;
                self.internal_state.hs_info.y_target_dir =
                    f32::from(car.team == Team::Blue) * 2.0 - 1.0;

                #[allow(clippy::eq_op)]
                if can_increase
                    && self.internal_state.hs_info.y_target_dir
                        != self.internal_state.hs_info.y_target_dir
                {
                    self.internal_state.hs_info.time_since_hit = 0.0;
                    self.internal_state.hs_info.cur_target_speed = heatseeker::MAX_SPEED.min(
                        self.internal_state.hs_info.cur_target_speed
                            + heatseeker::TARGET_SPEED_INCREMENT,
                    );
                }
            }
            GameMode::Dropshot => {
                let accumulated_hit_force = &mut self.internal_state.ds_info.accumulated_hit_force;
                let charge_level = &mut self.internal_state.ds_info.charge_level;

                let dir_from_car =
                    (self.internal_state.physics.pos - car.internal_state.physics.pos).normalize();
                let rel_vel_from_car =
                    car.internal_state.physics.vel - self.internal_state.physics.vel;
                let vel_info_ball = dir_from_car.dot(rel_vel_from_car);

                if vel_info_ball >= dropshot::MIN_CHARGE_HIT_SPEED {
                    *accumulated_hit_force += vel_info_ball;

                    if *accumulated_hit_force >= dropshot::MIN_ABSORBED_FORCE_FOR_SUPERCHARGE {
                        *charge_level = 3;
                    } else if *accumulated_hit_force >= dropshot::MIN_ABSORBED_FORCE_FOR_CHARGE {
                        *charge_level = 2;
                    }
                }

                if *charge_level > 1 {
                    self.internal_state.ds_info.y_target_dir =
                        f32::from(car.team == Team::Blue) * 2.0 - 1.0;
                }
            }
            _ => {}
        }
    }
}
