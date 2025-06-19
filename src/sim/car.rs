use super::{BallHitInfo, CarConfig, CarControls, CollisionMasks, MutatorConfig, PhysState};
use crate::{
    BT_TO_UU, GameMode, UU_TO_BT, UserInfoTypes,
    bullet::{
        collision::{
            broadphase::broadphase_proxy::CollisionFilterGroups,
            dispatch::collision_object::CollisionFlags,
            shapes::{
                box_shape::BoxShape, collision_shape::CollisionShapes,
                compound_shape::CompoundShape,
            },
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
            vehicle::{
                raycaster::VehicleRaycaster,
                vehicle_rl::{VehicleRL, VehicleTuning},
            },
        },
    },
    consts::{
        self, CAR_RESPAWN_LOCATION_AMOUNT, CAR_RESPAWN_LOCATIONS_DROPSHOT,
        CAR_RESPAWN_LOCATIONS_HOOPS, CAR_RESPAWN_LOCATIONS_SOCCAR, CAR_RESPAWN_Z,
        btvehicle::{
            MAX_SUSPENSION_TRAVEL, SUSPENSION_FORCE_SCALE_BACK, SUSPENSION_FORCE_SCALE_FRONT,
            SUSPENSION_STIFFNESS, WHEELS_DAMPING_COMPRESSION, WHEELS_DAMPING_RELAXATION,
        },
    },
};
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};
use std::{
    cell::RefCell,
    f32::{self, consts::PI},
    rc::Rc,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct CarContact {
    pub other_car_id: u32,
    pub cooldown_timer: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct CarState {
    pub physics: PhysState,
    pub tick_count_since_update: u64,
    /// True if 3 or more wheels have contact
    pub is_on_ground: bool,
    /// Whether each of the 4 wheels have contact
    /// First two are front
    /// If your car has 3 wheels, the 4th bool will always be false
    pub wheels_with_contact: [bool; 4],
    /// Whether we jumped to get into the air
    ///
    /// Can be false while airborne, if we left the ground with a flip reset
    pub has_jumped: bool,
    /// True if we have double jumped and are still in the air
    pub has_double_jumped: bool,
    /// True if we are in the air, and (have flipped or are currently flipping)
    pub has_flipped: bool,
    /// Relative torque direction of the flip
    ///
    /// Forward flip will have positive Y
    pub flip_rel_torque: Vec3A,
    /// When currently jumping, the time since we started jumping, else 0
    pub jump_time: f32,
    /// When currently flipping, the time since we started flipping, else 0
    pub flip_time: f32,
    /// True during a flip (not an auto-flip, and not after a flip)
    pub is_flipping: bool,
    /// True during a jump
    pub is_jumping: bool,
    /// Total time spent in the air
    pub air_time: f32,
    /// Time spent in the air once !is_jumping
    ///
    /// If we never jumped, it is 0
    pub air_time_since_jump: f32,
    /// Goes from 0 to 100
    pub boost: f32,
    /// Used for recharge boost, counts up from 0 on spawn (in seconds)
    pub time_since_boosted: f32,
    /// True if we boosted that tick
    ///
    /// There exists a minimum boosting time, thus why we must track boosting time
    pub is_boosting: bool,
    pub boosting_time: f32,
    pub is_supersonic: bool,
    /// Time spent supersonic, for checking with the supersonic maintain time
    pub supersonic_time: f32,
    /// This is a state variable due to the rise/fall rate of handbrake inputs
    pub handbrake_val: f32,
    pub is_auto_flipping: bool,
    /// Counts down when auto-flipping
    pub auto_flip_timer: f32,
    pub auto_flip_torque_scale: f32,
    pub world_contact_normal: Option<Vec3A>,
    pub car_contact: CarContact,
    pub is_demoed: bool,
    pub demo_respawn_timer: f32,
    pub ball_hit_info: Option<BallHitInfo>,
    /// Controls from last tick, set to this->controls after simulation
    pub last_controls: CarControls,
}

impl Default for CarState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarState {
    pub const DEFAULT: Self = Self {
        physics: PhysState {
            pos: Vec3A::new(0.0, 0.0, consts::CAR_SPAWN_REST_Z),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        tick_count_since_update: 0,
        is_on_ground: true,
        wheels_with_contact: [false; 4],
        has_jumped: false,
        has_double_jumped: false,
        has_flipped: false,
        flip_rel_torque: Vec3A::ZERO,
        jump_time: 0.0,
        flip_time: 0.0,
        is_flipping: false,
        is_jumping: false,
        air_time: 0.0,
        air_time_since_jump: 0.0,
        boost: consts::BOOST_SPAWN_AMOUNT,
        time_since_boosted: 0.0,
        is_boosting: false,
        boosting_time: 0.0,
        is_supersonic: false,
        supersonic_time: 0.0,
        handbrake_val: 0.0,
        is_auto_flipping: false,
        auto_flip_timer: 0.0,
        auto_flip_torque_scale: 0.0,
        world_contact_normal: None,
        car_contact: CarContact {
            other_car_id: 0,
            cooldown_timer: 0.0,
        },
        is_demoed: false,
        demo_respawn_timer: 0.0,
        ball_hit_info: None,
        last_controls: CarControls::DEFAULT,
    };

    pub const fn has_flip_or_jump(&self) -> bool {
        self.is_on_ground
            || (!self.has_flipped
                && !self.has_double_jumped
                && self.air_time_since_jump < consts::DOUBLEJUMP_MAX_DELAY)
    }

    pub const fn has_flip_reset(&self) -> bool {
        !self.is_on_ground && self.has_flip_or_jump() && !self.has_jumped
    }

    pub const fn got_flip_reset(&self) -> bool {
        !self.is_on_ground && !self.has_jumped
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Team {
    #[default]
    Blue,
    Orange,
}

impl Team {
    pub fn from_team_y(y: f32) -> Self {
        if y.is_sign_negative() {
            Self::Blue
        } else {
            Self::Orange
        }
    }

    pub fn into_team_y(self) -> f32 {
        f32::from(self as i8 * 2 - 1)
    }
}

pub struct Car {
    pub team: Team,
    /// The controls to simulate the car with
    pub controls: CarControls,
    pub(crate) config: CarConfig,
    bullet_vehicle: VehicleRL,
    pub(crate) rigid_body: Rc<RefCell<RigidBody>>,
    pub(crate) velocity_impulse_cache: Vec3A,
    pub(crate) internal_state: CarState,
}

impl Car {
    pub(crate) fn new(
        bullet_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        team: Team,
        config: CarConfig,
    ) -> Self {
        let child_hitbox_shape = BoxShape::new(config.hitbox_size * UU_TO_BT * 0.5);
        let local_inertia = child_hitbox_shape.calculate_local_intertia(consts::CAR_MASS_BT);

        let mut compound_shape = CompoundShape::new();
        let hitbox_offset = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation: config.hitbox_pos_offset * UU_TO_BT,
        };
        compound_shape.add_child_shape(hitbox_offset, child_hitbox_shape);

        let collision_shape = CollisionShapes::Compound(compound_shape);
        let mut info = RigidBodyConstructionInfo::new(consts::CAR_MASS_BT, collision_shape);
        info.friction = consts::CAR_COLLISION_FRICTION;
        info.restitution = consts::CAR_COLLISION_RESTITUTION;
        info.start_world_transform = Affine3A::IDENTITY;
        info.local_inertia = local_inertia;

        let body = RigidBody::new(info);
        let mut co = body.collision_object.borrow_mut();
        co.user_index = UserInfoTypes::Car;
        co.collision_flags |= CollisionFlags::CustomMaterialCallback as i32;
        drop(co);

        let rigid_body = Rc::new(RefCell::new(body));

        bullet_world.add_rigid_body(
            rigid_body.clone(),
            CollisionFilterGroups::DefaultFilter as i32 | CollisionMasks::DropshotFloor as i32,
            CollisionFilterGroups::AllFilter as i32,
        );

        let tuning = VehicleTuning {
            suspension_stiffness: SUSPENSION_STIFFNESS,
            suspension_compression: WHEELS_DAMPING_COMPRESSION,
            suspension_damping: WHEELS_DAMPING_RELAXATION,
            max_suspension_travel_cm: MAX_SUSPENSION_TRAVEL * UU_TO_BT * 100.0,
            max_suspension_force: f32::MAX,
            ..Default::default()
        };
        let raycaster = VehicleRaycaster::new(CollisionMasks::DropshotFloor as i32);
        let mut bullet_vehicle = VehicleRL::new(rigid_body.clone(), raycaster);

        let wheel_direction_cs = Vec3A::new(0.0, 0.0, -1.0);
        let wheel_axle_cs = Vec3A::new(0.0, -1.0, 0.0);

        for i in 0..4 {
            let front = i < 2;
            let left = i % 2 == 0;

            let wheels = if front {
                &config.front_wheels
            } else {
                &config.back_wheels
            };

            let radius = wheels.wheel_radius;
            let mut wheel_ray_start_offset = wheels.connection_point_offset;
            let suspension_rest_length = wheels.suspension_rest_length - MAX_SUSPENSION_TRAVEL;

            if left {
                wheel_ray_start_offset.y *= -1.0;
            }

            bullet_vehicle.add_wheel(
                wheel_ray_start_offset * UU_TO_BT,
                wheel_direction_cs,
                wheel_axle_cs,
                suspension_rest_length,
                radius * UU_TO_BT,
                &tuning,
                front,
            );

            bullet_vehicle.wheel_info[i].suspsension_force_scale = if front {
                SUSPENSION_FORCE_SCALE_FRONT
            } else {
                SUSPENSION_FORCE_SCALE_BACK
            };
        }

        Self {
            team,
            config,
            rigid_body,
            bullet_vehicle,
            controls: CarControls::DEFAULT,
            velocity_impulse_cache: Vec3A::ZERO,
            internal_state: CarState {
                boost: mutator_config.car_spawn_boost_amount,
                ..Default::default()
            },
        }
    }

    /// Get the forward direction as a unit vector
    pub const fn get_forward_dir(&self) -> Vec3A {
        self.internal_state.physics.rot_mat.x_axis
    }

    /// Get the rightward direction as a unit vector
    pub const fn get_right_dir(&self) -> Vec3A {
        self.internal_state.physics.rot_mat.y_axis
    }

    /// Get the upward direction as a unit vector
    pub const fn get_up_dir(&self) -> Vec3A {
        self.internal_state.physics.rot_mat.z_axis
    }

    /// Configuration for this car
    pub const fn config(&self) -> &CarConfig {
        &self.config
    }

    /// - `respawn_delay` by default is `rocketsim::consts::DEMO_RESPAWN_TIME`
    pub const fn demolish(&mut self, respawn_delay: f32) {
        self.internal_state.is_demoed = true;
        self.internal_state.demo_respawn_timer = respawn_delay;
    }

    /// - `boost_amount` by default is `rocketsim::consts::BOOST_RESPAWN_AMOUNT`
    ///
    /// Respawn the car, called after we have been demolished and waited for the respawn timer
    pub fn respawn(&mut self, game_mode: GameMode, boost_amount: f32) {
        let spawn_pos_index = fastrand::usize(0..CAR_RESPAWN_LOCATION_AMOUNT);
        let spawn_pos = match game_mode {
            GameMode::Hoops => &CAR_RESPAWN_LOCATIONS_HOOPS,
            GameMode::Dropshot => &CAR_RESPAWN_LOCATIONS_DROPSHOT,
            _ => &CAR_RESPAWN_LOCATIONS_SOCCAR,
        }[spawn_pos_index];

        let new_state = CarState {
            physics: PhysState {
                pos: Vec3A::new(
                    spawn_pos.x,
                    spawn_pos.y * -self.team.into_team_y(),
                    CAR_RESPAWN_Z,
                ),
                rot_mat: Mat3A::from_euler(
                    EulerRot::YZX,
                    0.0,
                    spawn_pos.yaw_ang + if self.team == Team::Blue { 0.0 } else { PI },
                    0.0,
                ),
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
            },
            boost: boost_amount,
            ..Default::default()
        };

        self.set_state(&new_state);
    }

    pub fn get_state(&self) -> CarState {
        let rb = self.rigid_body.borrow();

        let mut internal_state = self.internal_state;
        internal_state.physics.pos = rb
            .collision_object
            .borrow()
            .get_world_transform()
            .translation
            * BT_TO_UU;
        internal_state.physics.vel = rb.linear_velocity * BT_TO_UU;
        internal_state.physics.ang_vel = rb.angular_velocity;

        internal_state
    }

    pub fn set_state(&mut self, state: &CarState) {
        let mut rb = self.rigid_body.borrow_mut();

        rb.collision_object
            .borrow_mut()
            .set_world_transform(Affine3A {
                matrix3: state.physics.rot_mat,
                translation: state.physics.pos * UU_TO_BT,
            });

        rb.linear_velocity = state.physics.vel * UU_TO_BT;
        rb.angular_velocity = state.physics.ang_vel;
        rb.update_inertia_tensor();

        self.velocity_impulse_cache = Vec3A::ZERO;
        self.internal_state.tick_count_since_update = 0;
    }
}
