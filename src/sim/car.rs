use super::{BallHitInfo, CarConfig, CarControls, CollisionMasks, MutatorConfig, PhysState};
use crate::{
    BT_TO_UU, GameMode, UU_TO_BT, UserInfoTypes,
    bullet::{
        collision::{
            broadphase::broadphase_proxy::CollisionFilterGroups,
            dispatch::{
                collision_object::{ACTIVE_TAG, CollisionFlags, DISABLE_SIMULATION},
                collision_world::CollisionWorld,
            },
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
        btvehicle::{
            MAX_SUSPENSION_TRAVEL, SUSPENSION_FORCE_SCALE_BACK, SUSPENSION_FORCE_SCALE_FRONT,
            SUSPENSION_STIFFNESS, WHEELS_DAMPING_COMPRESSION, WHEELS_DAMPING_RELAXATION,
        },
        *,
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
            pos: Vec3A::new(0.0, 0.0, CAR_SPAWN_REST_Z),
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
        boost: BOOST_SPAWN_AMOUNT,
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
                && self.air_time_since_jump < DOUBLEJUMP_MAX_DELAY)
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
        let local_inertia = child_hitbox_shape.calculate_local_intertia(CAR_MASS_BT);

        let mut compound_shape = CompoundShape::new();
        let hitbox_offset = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation: config.hitbox_pos_offset * UU_TO_BT,
        };
        compound_shape.add_child_shape(hitbox_offset, child_hitbox_shape);

        let collision_shape = CollisionShapes::Compound(compound_shape);
        let mut info = RigidBodyConstructionInfo::new(CAR_MASS_BT, collision_shape);
        info.friction = CAR_COLLISION_FRICTION;
        info.restitution = CAR_COLLISION_RESTITUTION;
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

    fn update_wheels(
        &mut self,
        tick_time: f32,
        mutator_config: &MutatorConfig,
        num_wheels_in_contact: u8,
        forward_speed_uu: f32,
    ) {
        let abs_forward_speed_uu = forward_speed_uu.abs();

        let mut wheels_have_world_contact = false;
        for wheel in &self.bullet_vehicle.wheel_info {
            wheels_have_world_contact |= wheel.is_in_contact_with_world;
        }

        self.internal_state.handbrake_val += if self.controls.handbrake {
            POWERSLIDE_RISE_RATE * tick_time
        } else {
            -POWERSLIDE_FALL_RATE * tick_time
        };
        self.internal_state.handbrake_val = self.internal_state.handbrake_val.clamp(0.0, 1.0);

        let mut real_brake = 0.0;
        let real_throttle = if self.controls.boost && self.internal_state.boost > 0.0 {
            1.0
        } else {
            self.controls.throttle
        };

        let mut engine_throttle = real_throttle;
        if !self.controls.handbrake {
            if real_throttle.abs() >= THROTTLE_DEADZONE {
                if abs_forward_speed_uu > STOPPING_FORWARD_VEL
                    && real_throttle.signum() != forward_speed_uu.signum()
                {
                    real_brake = 1.0;

                    if abs_forward_speed_uu > BRAKING_NO_THROTTLE_SPEED_THRESH {
                        engine_throttle = 0.0;
                    }
                }
            } else {
                engine_throttle = 0.0;
                real_brake = if abs_forward_speed_uu < STOPPING_FORWARD_VEL {
                    1.0
                } else {
                    COASTING_BRAKE_FACTOR
                };
            }
        }

        let mut drive_speed_scale =
            DRIVE_SPEED_TORQUE_FACTOR_CURVE.get_output(abs_forward_speed_uu, None);
        if num_wheels_in_contact < 3 {
            drive_speed_scale /= 4.0;
        }

        let drive_engine_force =
            engine_throttle * const { THROTTLE_TORQUE_AMOUNT * UU_TO_BT } * drive_speed_scale;
        let drive_brake_force = real_brake * const { BRAKE_TORQUE_AMOUNT * UU_TO_BT };
        for wheel in &mut self.bullet_vehicle.wheel_info {
            wheel.wheel_info.engine_force = drive_engine_force;
            wheel.wheel_info.brake = drive_brake_force;
        }

        let mut steer_angle = if self.config.three_wheels {
            STEER_ANGLE_FROM_SPEED_CURVE_THREEWHEEL.get_output(abs_forward_speed_uu, None)
        } else {
            STEER_ANGLE_FROM_SPEED_CURVE.get_output(abs_forward_speed_uu, None)
        };
        if self.internal_state.handbrake_val != 0.0 {
            steer_angle += (POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE
                .get_output(abs_forward_speed_uu, None)
                - steer_angle)
                * self.internal_state.handbrake_val
        }

        steer_angle *= self.controls.steer;
        self.bullet_vehicle.wheel_info[0].steer_angle = steer_angle;
        self.bullet_vehicle.wheel_info[1].steer_angle = steer_angle;

        for wheel in &mut self.bullet_vehicle.wheel_info {
            if let Some(ground_obj) = wheel.wheel_info.raycast_info.ground_object.as_ref() {
                todo!("wheel-ground contact")
            }
        }

        if wheels_have_world_contact {
            let upwards_dir = self.bullet_vehicle.get_upwards_dir_from_wheel_contacts();

            let full_stick = real_throttle != 0.0 || abs_forward_speed_uu > STOPPING_FORWARD_VEL;
            let mut sticky_force_scale = if self.config.three_wheels { 0.0 } else { 0.5 };
            if full_stick {
                sticky_force_scale += 1.0 - upwards_dir.z.abs();
            }

            self.rigid_body.borrow_mut().apply_central_force(
                upwards_dir * sticky_force_scale * const { GRAVITY_Z * UU_TO_BT * CAR_MASS_BT },
            );
        }
    }

    fn update_air_torque(&mut self, update_air_control: bool) {
        let dir_pitch = -self.get_right_dir();
        let dir_yaw = self.get_up_dir();
        let dir_roll = -self.get_forward_dir();

        if self.internal_state.is_flipping {
            self.internal_state.is_flipping =
                self.internal_state.has_flipped && self.internal_state.flip_time < FLIP_TORQUE_TIME;
        }

        let mut rb = self.rigid_body.borrow_mut();
        let mut do_air_control = false;
        if self.internal_state.is_flipping {
            if self.internal_state.flip_rel_torque == Vec3A::ZERO {
                do_air_control = true;
            } else {
                let mut rel_dodge_torque = self.internal_state.flip_rel_torque;

                let mut pitch_scale = 1.0;
                if rel_dodge_torque.y != 0.0
                    && self.controls.pitch != 0.0
                    && rel_dodge_torque.y.signum() == self.controls.pitch.signum()
                {
                    pitch_scale = 1.0 - self.controls.pitch.abs().min(1.0);
                    do_air_control = true;
                }

                rel_dodge_torque.y *= pitch_scale;
                let dodge_torque =
                    rel_dodge_torque * const { Vec3A::new(FLIP_TORQUE_X, FLIP_TORQUE_Y, 0.0) };

                let rb_torque = rb.inv_inertia_tensor_world.inverse()
                    * rb.collision_object.borrow().get_world_transform().matrix3
                    * dodge_torque;
                rb.apply_central_force(rb_torque);
            }
        } else {
            do_air_control = true;
        }

        do_air_control &= !self.internal_state.is_auto_flipping;
        do_air_control &= update_air_control;
        if do_air_control {
            let mut pitch_torque_scale = 1.0;
            let torque = if self.controls.pitch != 0.0
                || self.controls.yaw != 0.0
                || self.controls.roll != 0.0
            {
                if self.internal_state.is_flipping {
                    pitch_torque_scale = 0.0;
                } else if self.internal_state.has_flipped
                    && self.internal_state.flip_time
                        < const { FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME }
                {
                    pitch_torque_scale = 0.0;
                }

                self.controls.pitch * dir_pitch * pitch_torque_scale * CAR_AIR_CONTROL_TORQUE.x
                    + self.controls.yaw * dir_yaw * CAR_AIR_CONTROL_TORQUE.y
                    + self.controls.roll * dir_roll * CAR_AIR_CONTROL_TORQUE.z
            } else {
                Vec3A::ZERO
            };

            let ang_vel = rb.angular_velocity;

            let damp_pitch = dir_pitch.dot(ang_vel)
                * CAR_AIR_CONTROL_DAMPING.x
                * (1.0 - (self.controls.pitch * pitch_torque_scale).abs());
            let damp_yaw =
                dir_yaw.dot(ang_vel) * CAR_AIR_CONTROL_DAMPING.y * (1.0 - self.controls.yaw.abs());
            let damp_roll = dir_roll.dot(ang_vel) * CAR_AIR_CONTROL_DAMPING.z;

            let damping = dir_yaw * damp_yaw + dir_pitch * damp_pitch + dir_roll * damp_roll;

            let rb_torque =
                rb.inv_inertia_tensor_world.inverse() * (torque - damping) * CAR_TORQUE_SCALE;
            rb.apply_torque(rb_torque);
        }

        if self.controls.throttle != 0.0 {
            rb.apply_central_force(
                self.get_forward_dir()
                    * self.controls.throttle
                    * const { THROTTLE_AIR_ACCEL * UU_TO_BT * CAR_MASS_BT },
            );
        }
    }

    fn update_jump(&mut self, tick_time: f32, mutator_config: &MutatorConfig, jump_pressed: bool) {
        if self.internal_state.is_on_ground && self.internal_state.is_jumping {
            if self.internal_state.has_jumped
                && self.internal_state.jump_time < const { JUMP_MIN_TIME + JUMP_RESET_TIME_PAD }
            {
                // Don't reset the jump just yet, we might still be leaving the ground
                // This fixes the bug where jump is reset before we actually leave the ground after a minimum-time jump
                // TODO: RL does something similar to this time-pad, but not exactly the same
            } else {
                self.internal_state.has_jumped = false;
                self.internal_state.jump_time = 0.0;
            }
        }

        let mut rb = self.rigid_body.borrow_mut();
        if self.internal_state.is_jumping {
            self.internal_state.is_jumping = self.internal_state.jump_time < JUMP_MIN_TIME
                || (self.controls.jump && self.internal_state.jump_time < JUMP_MAX_TIME);
        } else if self.internal_state.is_on_ground && jump_pressed {
            self.internal_state.is_jumping = true;
            self.internal_state.jump_time = 0.0;
            let jump_start_force = self.get_up_dir()
                * mutator_config.jump_immediate_force
                * const { UU_TO_BT * CAR_MASS_BT };
            rb.apply_central_impulse(jump_start_force);
        }

        if self.internal_state.is_jumping {
            self.internal_state.has_jumped = true;

            let mut total_jump_force = self.get_up_dir() * mutator_config.jump_accel;
            if self.internal_state.jump_time < JUMP_MIN_TIME {
                const JUMP_PRE_MIN_ACCEL_SCALE: f32 = 0.62;
                total_jump_force *= JUMP_PRE_MIN_ACCEL_SCALE;
            }

            rb.apply_central_force(total_jump_force * const { UU_TO_BT * CAR_MASS_BT });
        }

        if self.internal_state.is_jumping || self.internal_state.has_jumped {
            self.internal_state.jump_time += tick_time;
        }
    }

    fn update_auto_flip(&mut self, tick_time: f32, jump_pressed: bool) {
        if jump_pressed
            && self
                .internal_state
                .world_contact_normal
                .is_some_and(|world_contact_normal| {
                    world_contact_normal.z > CAR_AUTOFLIP_NORMZ_THRESH
                })
        {
            let (_, _, roll) = self.internal_state.physics.rot_mat.to_euler(EulerRot::YZX);
            let abs_roll = roll.abs();
            if abs_roll > CAR_AUTOFLIP_ROLL_THRESH {
                self.internal_state.auto_flip_timer = CAR_AUTOFLIP_TIME * (abs_roll / PI);
                self.internal_state.auto_flip_torque_scale = roll.signum();
                self.internal_state.is_auto_flipping = true;

                self.rigid_body.borrow_mut().apply_central_impulse(
                    -self.get_up_dir() * const { CAR_AUTOFLIP_IMPULSE * UU_TO_BT * CAR_MASS_BT },
                );
            }
        }

        if self.internal_state.is_auto_flipping {
            if self.internal_state.auto_flip_timer <= 0.0 {
                self.internal_state.is_auto_flipping = false;
                self.internal_state.auto_flip_timer = 0.0;
            } else {
                self.rigid_body.borrow_mut().angular_velocity += self.get_forward_dir()
                    * CAR_AUTOFLIP_TORQUE
                    * self.internal_state.auto_flip_torque_scale
                    * tick_time;
                self.internal_state.auto_flip_timer -= tick_time;
            }
        }
    }

    fn update_double_jump_or_flip(
        &mut self,
        tick_time: f32,
        mutator_config: &MutatorConfig,
        jump_pressed: bool,
        forward_speed_uu: f32,
    ) {
        if self.internal_state.is_on_ground {
            self.internal_state.has_double_jumped = false;
            self.internal_state.has_flipped = false;
            self.internal_state.air_time = 0.0;
            self.internal_state.air_time_since_jump = 0.0;
            self.internal_state.flip_time = 0.0;
            return;
        }

        self.internal_state.air_time += tick_time;

        if self.internal_state.has_jumped && !self.internal_state.is_jumping {
            self.internal_state.air_time_since_jump += tick_time
        } else {
            self.internal_state.air_time_since_jump = 0.0;
        }

        let mut rb = self.rigid_body.borrow_mut();
        if jump_pressed && self.internal_state.air_time_since_jump < DOUBLEJUMP_MAX_DELAY {
            let input_magnitude =
                self.controls.yaw.abs() + self.controls.pitch.abs() + self.controls.roll.abs();
            let is_flip_input = input_magnitude >= self.config.dodge_deadzone;

            let can_use = !self.internal_state.is_auto_flipping
                && !self.internal_state.has_double_jumped
                && !self.internal_state.has_flipped
                || if is_flip_input {
                    mutator_config.unlimited_flips
                } else {
                    mutator_config.unlimited_double_jumps
                };

            if can_use {
                if is_flip_input {
                    self.internal_state.flip_time = 0.0;
                    self.internal_state.has_flipped = true;
                    self.internal_state.is_flipping = true;

                    let forward_speed_ratio = forward_speed_uu.abs() / CAR_MAX_SPEED;
                    let mut dodge_dir = Vec3A::new(
                        -self.controls.pitch,
                        self.controls.yaw + self.controls.roll,
                        0.0,
                    );

                    self.internal_state.flip_rel_torque =
                        Vec3A::new(-dodge_dir.y, dodge_dir.x, 0.0);

                    if dodge_dir.x.abs() < 0.1 {
                        dodge_dir.x = 0.0;
                    }

                    if dodge_dir.y.abs() < 0.1 {
                        dodge_dir.y = 0.0;
                    }

                    if dodge_dir.length_squared() > const { f32::EPSILON * f32::EPSILON } {
                        let should_dodge_backwards = if forward_speed_uu.abs() < 100. {
                            dodge_dir.x.is_sign_negative()
                        } else {
                            dodge_dir.x.signum() != forward_speed_uu.signum()
                        };

                        let max_speed_scale_x = if should_dodge_backwards {
                            FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE
                        } else {
                            FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE
                        };

                        let mut initial_dodge_vel = dodge_dir * FLIP_INITIAL_VEL_SCALE;
                        initial_dodge_vel.x *=
                            ((max_speed_scale_x - 1.) * forward_speed_ratio) + 1.;
                        initial_dodge_vel.y *=
                            ((FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1.) * forward_speed_ratio) + 1.;
                        if should_dodge_backwards {
                            initial_dodge_vel.x *= FLIP_BACKWARD_IMPULSE_SCALE_X;
                        }

                        let forward_dir_2d = self.get_forward_dir().with_z(0.0).normalize();
                        let right_dir_2d = Vec3A::new(-forward_dir_2d.y, forward_dir_2d.x, 0.0);
                        let final_delta_vel =
                            initial_dodge_vel * forward_dir_2d + initial_dodge_vel.y * right_dir_2d;

                        rb.apply_central_impulse(
                            final_delta_vel * const { UU_TO_BT * CAR_MASS_BT },
                        );
                    }
                } else {
                    let jump_start_force =
                        self.get_up_dir() * const { JUMP_IMMEDIATE_FORCE * UU_TO_BT * CAR_MASS_BT };
                    rb.apply_central_impulse(jump_start_force);
                    self.internal_state.has_double_jumped = true;
                }
            }
        }

        if self.internal_state.is_flipping {
            self.internal_state.flip_time += tick_time;
            if self.internal_state.flip_time <= FLIP_TORQUE_TIME
                && self.internal_state.flip_time >= FLIP_Z_DAMP_START
                && (rb.linear_velocity.z < 0.0 || self.internal_state.flip_time < FLIP_Z_DAMP_END)
            {
                rb.linear_velocity.z *= (1. - FLIP_Z_DAMP_120).powf(tick_time / (1. / 120.));
            }
        } else if self.internal_state.has_flipped {
            self.internal_state.flip_time += tick_time;
        }
    }

    fn update_auto_roll(&mut self, num_wheels_in_contact: u8) {
        let ground_up_dir = if num_wheels_in_contact > 0 {
            self.bullet_vehicle.get_upwards_dir_from_wheel_contacts()
        } else {
            self.internal_state.world_contact_normal.unwrap()
        };

        let ground_down_dir = -ground_up_dir;

        let forward_dir = self.get_forward_dir();
        let right_dir = self.get_right_dir();

        let cross_right_dir = ground_up_dir.cross(forward_dir);
        let cross_forward_dir = ground_down_dir.cross(cross_right_dir);

        let right_torque_factor = 1.0 - right_dir.dot(cross_right_dir).clamp(0.0, 1.0);
        let forward_torque_factor = 1.0 - forward_dir.dot(cross_forward_dir).clamp(0.0, 1.0);

        let torque_dir_right = forward_dir * -right_dir.dot(ground_up_dir).signum();
        let torque_dir_forward = right_dir * forward_dir.dot(ground_up_dir).signum();

        let torque_right = torque_dir_right * right_torque_factor;
        let torque_forward = torque_dir_forward * forward_torque_factor;

        let mut rb = self.rigid_body.borrow_mut();
        rb.apply_central_force(
            ground_down_dir * const { CAR_AUTOROLL_FORCE * UU_TO_BT * CAR_MASS_BT },
        );

        let rb_torque = rb.inv_inertia_tensor_world.inverse()
            * (torque_forward + torque_right)
            * CAR_AUTOROLL_TORQUE;
        rb.apply_torque(rb_torque);
    }

    fn update_boost(&mut self, tick_time: f32, mutator_config: &MutatorConfig) {
        self.internal_state.is_boosting = if self.internal_state.boost > 0.0 {
            self.controls.boost
                || (self.internal_state.is_boosting
                    && self.internal_state.boosting_time < BOOST_MIN_TIME)
        } else {
            false
        };

        if self.internal_state.is_boosting {
            self.internal_state.boosting_time += tick_time;
            self.internal_state.time_since_boosted = 0.0;
            self.internal_state.boost -= mutator_config.boost_used_per_second * tick_time;

            let accel = if self.internal_state.is_on_ground {
                mutator_config.boost_accel_ground
            } else {
                mutator_config.boost_accel_air
            };

            self.rigid_body.borrow_mut().apply_central_force(
                accel * self.get_forward_dir() * const { UU_TO_BT * CAR_MASS_BT },
            );
        } else {
            self.internal_state.boosting_time = 0.0;
            self.internal_state.time_since_boosted += tick_time;

            if mutator_config.recharge_boost_enabled
                && self.internal_state.time_since_boosted >= mutator_config.recharge_boost_delay
            {
                self.internal_state.boost += mutator_config.recharge_boost_per_second * tick_time;
            }
        }

        self.internal_state.boost = self.internal_state.boost.clamp(0.0, BOOST_MAX);
    }

    pub(crate) fn pre_tick_update(
        &mut self,
        collision_world: &CollisionWorld,
        game_mode: GameMode,
        tick_time: f32,
        mutator_config: &MutatorConfig,
    ) {
        debug_assert!(
            self.bullet_vehicle.get_num_wheels() == 4 || self.bullet_vehicle.get_num_wheels() == 3
        );

        if self.internal_state.is_demoed {
            self.internal_state.demo_respawn_timer =
                (self.internal_state.demo_respawn_timer - tick_time).max(0.0);
            if self.internal_state.demo_respawn_timer == 0.0 {
                self.respawn(game_mode, mutator_config.car_spawn_boost_amount);
            }

            let rb = self.rigid_body.borrow();
            let mut co = rb.collision_object.borrow_mut();
            co.activation_state_1 = DISABLE_SIMULATION;
            co.collision_flags |= CollisionFlags::NoContactResponse as i32;
        } else {
            let rb = self.rigid_body.borrow();
            let mut co = rb.collision_object.borrow_mut();
            co.activation_state_1 = ACTIVE_TAG;
            co.collision_flags &= !(CollisionFlags::NoContactResponse as i32);
        }

        if self.internal_state.is_demoed {
            return;
        }

        self.controls.clamp_fix();

        // Do first part of the btVehicleRL update (update wheel transforms, do traces, calculate friction impulses)
        self.bullet_vehicle
            .update_vehicle_first(collision_world, tick_time);

        let jump_pressed = self.controls.jump && !self.internal_state.last_controls.jump;

        let mut num_wheels_in_contact = 0u8;
        for (wheel, has_contact) in self
            .bullet_vehicle
            .wheel_info
            .iter()
            .zip(&mut self.internal_state.wheels_with_contact)
        {
            let in_contact = wheel.wheel_info.raycast_info.is_in_contact;
            *has_contact = in_contact;
            num_wheels_in_contact += u8::from(in_contact);
        }

        self.internal_state.is_on_ground = num_wheels_in_contact >= 3;

        let forward_speed_uu = self.bullet_vehicle.get_forward_speed() * BT_TO_UU;
        self.update_wheels(
            tick_time,
            mutator_config,
            num_wheels_in_contact,
            forward_speed_uu,
        );

        if !self.internal_state.is_on_ground {
            self.update_air_torque(num_wheels_in_contact == 0);
        } else {
            self.internal_state.is_flipping = false;
        }

        self.update_jump(tick_time, mutator_config, jump_pressed);
        self.update_auto_flip(tick_time, jump_pressed);
        self.update_double_jump_or_flip(tick_time, mutator_config, jump_pressed, forward_speed_uu);

        if self.controls.throttle != 0.0 && (0 < num_wheels_in_contact && num_wheels_in_contact < 4)
            || self.internal_state.world_contact_normal.is_some()
        {
            self.update_auto_roll(num_wheels_in_contact);
        }

        self.internal_state.world_contact_normal = None;

        self.bullet_vehicle.update_vehicle_second(tick_time);
        self.update_boost(tick_time, mutator_config);
    }

    pub(crate) fn post_tick_update(&mut self, tick_time: f32) {
        if self.internal_state.is_demoed {
            return;
        }

        let rb = self.rigid_body.borrow();
        let co = rb.collision_object.borrow();

        self.internal_state.physics.rot_mat = co.get_world_transform().matrix3;

        let speed_squared = (rb.linear_velocity * BT_TO_UU).length_squared();
        self.internal_state.is_supersonic = speed_squared
            >= if self.internal_state.is_supersonic
                && self.internal_state.supersonic_time < SUPERSONIC_MAINTAIN_MAX_TIME
            {
                const { SUPERSONIC_MAINTAIN_MIN_SPEED * SUPERSONIC_MAINTAIN_MIN_SPEED }
            } else {
                const { SUPERSONIC_START_SPEED * SUPERSONIC_START_SPEED }
            };

        if self.internal_state.is_supersonic {
            self.internal_state.supersonic_time += tick_time;
        } else {
            self.internal_state.supersonic_time = 0.0;
        }

        if self.internal_state.car_contact.cooldown_timer > 0.0 {
            self.internal_state.car_contact.cooldown_timer =
                (self.internal_state.car_contact.cooldown_timer - tick_time).max(0.0);
        }

        self.internal_state.last_controls = self.controls;
    }

    pub(crate) fn finish_physics_tick(&mut self) {
        const MAX_SPEED: f32 = CAR_MAX_SPEED * UU_TO_BT;

        if self.internal_state.is_demoed {
            return;
        }

        let mut rb = self.rigid_body.borrow_mut();
        if self.velocity_impulse_cache != Vec3A::ZERO {
            rb.linear_velocity += self.velocity_impulse_cache;
            self.velocity_impulse_cache = Vec3A::ZERO;
        }

        let vel = &mut rb.linear_velocity;
        if vel.length_squared() > const { MAX_SPEED * MAX_SPEED } {
            *vel = vel.normalize() * MAX_SPEED;
        }

        let ang_vel = &mut rb.angular_velocity;
        if ang_vel.length_squared() > const { CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED } {
            *ang_vel = ang_vel.normalize() * CAR_MAX_ANG_SPEED;
        }

        self.internal_state.tick_count_since_update += 1;
    }
}
