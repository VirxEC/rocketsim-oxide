use std::{f32::consts::PI, iter::repeat_n, mem};

use ahash::AHashMap;
use arrayvec::ArrayVec;
use fastrand::Rng;
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};

use super::{Ball, BoostPadConfig, Car, CarConfig, CarState, MutatorConfig, PhysState, Team};
use crate::{
    ARENA_COLLISION_SHAPES, ArenaConfig, ArenaMemWeightMode, BoostPadGrid, GameMode,
    bullet::{
        collision::{
            broadphase::{GridBroadphase, HashedOverlappingPairCache},
            dispatch::{
                collision_dispatcher::CollisionDispatcher,
                collision_object::{ActivationState, CollisionObject},
                internal_edge_utility::adjust_internal_edge_contacts,
            },
            narrowphase::{
                manifold_point::ManifoldPoint, persistent_manifold::ContactAddedCallback,
            },
            shapes::{collision_shape::CollisionShapes, static_plane_shape::StaticPlaneShape},
        },
        dynamics::{
            constraint_solver::sequential_impulse_constraint_solver::SequentialImpulseConstraintSolver,
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
    },
    consts,
    consts::{BT_TO_UU, UU_TO_BT},
    sim::{
        BallState, BoostPad, BoostPadInfo, CarContact, CarInfo, DemoMode, GameState, UserInfoTypes,
        collision_masks::CollisionMasks,
    },
};

/// The arena's inner data, separated to prevent circular reference issues
struct ArenaInner {
    ball: Ball,
    cars: AHashMap<u64, Car>,
    tick_count: u64,
    game_mode: GameMode,
    mutator_config: MutatorConfig,
    boost_pad_grid: BoostPadGrid,
}

impl ArenaInner {
    fn on_car_ball_collision(
        &mut self,
        car_id: u64,
        manifold_point: &mut ManifoldPoint,
        ball_is_body_a: bool,
    ) {
        manifold_point.combined_friction = consts::car::HIT_BALL_COEFS.friction;
        manifold_point.combined_restitution = consts::car::HIT_BALL_COEFS.restitution;

        let rel_ball_pos = if ball_is_body_a {
            manifold_point.local_point_a
        } else {
            manifold_point.local_point_b
        };

        let car = self.cars.get_mut(&car_id).unwrap();
        self.ball.on_hit(
            car,
            rel_ball_pos,
            self.game_mode,
            &self.mutator_config,
            self.tick_count,
        );
    }

    fn on_car_world_collision(&mut self, car_id: u64, manifold_point: &mut ManifoldPoint) {
        let car = self.cars.get_mut(&car_id).unwrap();
        car.internal_state.world_contact_normal = Some(manifold_point.normal_world_on_b);

        manifold_point.combined_friction = self.mutator_config.car_world_friction;
        manifold_point.combined_restitution = self.mutator_config.car_world_restitution;
    }

    fn on_car_car_collision(
        &mut self,
        mut car_1_id: u64,
        mut car_2_id: u64,
        manifold_point: &mut ManifoldPoint,
    ) {
        manifold_point.combined_friction = consts::car::HIT_CAR_COEFS.friction;
        manifold_point.combined_restitution = consts::car::HIT_CAR_COEFS.restitution;

        debug_assert_ne!(car_1_id, car_2_id);
        let [car_1, car_2] =
            unsafe { self.cars.get_disjoint_unchecked_mut([&car_1_id, &car_2_id]) };
        let mut car_1 = car_1.unwrap();
        let mut car_2 = car_2.unwrap();

        if car_1.internal_state.is_demoed || car_2.internal_state.is_demoed {
            return;
        }

        // Test collision both ways
        for is_swapped in [false, true] {
            if is_swapped {
                mem::swap(&mut car_1, &mut car_2);
                mem::swap(&mut car_1_id, &mut car_2_id);
            }

            let state_1 = &car_1.internal_state;
            let state_2 = &car_2.internal_state;

            if let Some(car_contact) = state_1.car_contact
                && car_contact.other_car_id == car_2_id
                && car_contact.cooldown_timer > 0.0
            {
                // In cooldown
                continue;
            }

            let delta_pos = state_2.phys.pos - state_1.phys.pos;
            if state_1.phys.vel.dot(delta_pos) < 0.0 {
                // Moving away from the other car
                continue;
            }

            let vel_dir = state_1.phys.vel.normalize_or_zero();
            let dir_to_other_car = delta_pos.normalize();

            let speed_towards_other_car = state_1.phys.vel.dot(dir_to_other_car);
            let other_car_away_speed = state_2.phys.vel.dot(vel_dir);
            if speed_towards_other_car <= other_car_away_speed {
                // Going towards other car slower than they're going away
                continue;
            }

            let local_point = if is_swapped {
                manifold_point.local_point_b
            } else {
                manifold_point.local_point_a
            };

            let hit_with_bumper = local_point.x * BT_TO_UU > consts::car::bump::MIN_FORWARD_DIST;
            if !hit_with_bumper {
                // Didn't hit with bumper
                continue;
            }

            let mut is_demo = match self.mutator_config.demo_mode {
                DemoMode::OnContact => true,
                DemoMode::Disabled => false,
                DemoMode::Normal => state_1.is_supersonic,
            };
            if is_demo && !self.mutator_config.enable_team_demos {
                is_demo = car_1.team != car_2.team;
            }

            if is_demo {
                car_2.demolish(self.mutator_config.respawn_delay);
            } else {
                let ground_hit = state_2.is_on_ground;
                let base_scale = if ground_hit {
                    consts::curves::BUMP_VEL_AMOUNT_GROUND
                } else {
                    consts::curves::BUMP_VEL_AMOUNT_AIR
                }
                .get_output(speed_towards_other_car);

                let hit_up_dir = if state_2.is_on_ground {
                    state_2.phys.rot_mat.z_axis
                } else {
                    Vec3A::Z
                };

                let bump_impulse = vel_dir * base_scale
                    + hit_up_dir
                        * consts::curves::BUMP_UPWARD_VEL_AMOUNT
                            .get_output(speed_towards_other_car)
                        * self.mutator_config.bump_force_scale;

                car_2.velocity_impulse_cache += bump_impulse * UU_TO_BT;
            }

            car_1.internal_state.car_contact = Some(CarContact {
                other_car_id: car_2_id,
                cooldown_timer: self.mutator_config.bump_cooldown_time,
            });

            // TODO: car bump callback
        }
    }
}

impl ContactAddedCallback for ArenaInner {
    fn callback<'a>(
        &mut self,
        contact_point: &mut ManifoldPoint,
        mut body_a: &'a CollisionObject,
        mut body_b: &'a CollisionObject,
    ) {
        debug_assert!(body_a.has_contact_response() || body_b.has_contact_response());

        let should_swap = if body_a.user_index != UserInfoTypes::None
            && body_b.user_index != UserInfoTypes::None
        {
            body_a.user_index > body_b.user_index
        } else {
            body_b.user_index != UserInfoTypes::None
        };

        if should_swap {
            mem::swap(&mut body_a, &mut body_b);
        }

        let user_index_a = body_a.user_index;
        let user_index_b = body_b.user_index;

        if user_index_a == UserInfoTypes::Car {
            match user_index_b {
                UserInfoTypes::Ball => {
                    self.on_car_ball_collision(body_a.user_pointer, contact_point, should_swap);
                }
                UserInfoTypes::Car => self.on_car_car_collision(
                    body_a.user_pointer,
                    body_b.user_pointer,
                    contact_point,
                ),
                _ => self.on_car_world_collision(body_a.user_pointer, contact_point),
            }
        } else if user_index_a == UserInfoTypes::Ball {
            if user_index_b == UserInfoTypes::DropshotTile {
                todo!()
            } else if user_index_b == UserInfoTypes::None {
                contact_point.is_special = true;
            }
        }

        if should_swap {
            mem::swap(&mut body_a, &mut body_b);
        }

        let index = if should_swap {
            contact_point.index_0
        } else {
            contact_point.index_1
        };

        adjust_internal_edge_contacts(contact_point, body_a, index as usize);
    }
}

pub type GoalScoredCallback = fn(&mut Arena, Team);

pub struct Arena<D = ()> {
    rng: Rng,
    tick_time: f32,
    last_car_id: u64,
    config: ArenaConfig,
    bullet_world: DiscreteDynamicsWorld,
    data: ArenaInner,
    /// Callback that is called when a goal is scored.
    ///
    /// By default, this calls `Arena::reset_to_random_kickoff`.
    pub goal_scored_callback: Option<GoalScoredCallback>,
    pub user_data: Option<D>,
}

impl Arena {
    #[must_use]
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(game_mode, ArenaConfig::DEFAULT, 120)
    }

    pub fn new_with_config(game_mode: GameMode, config: ArenaConfig, tick_rate: u8) -> Self {
        assert!(
            (15..=120).contains(&tick_rate),
            "tick_rate must be between 15 and 120"
        );

        let mutator_config = MutatorConfig::new(game_mode);

        let collision_dispatcher = CollisionDispatcher::default();
        let constraint_solver = SequentialImpulseConstraintSolver::default();
        let overlapping_pair_cache = HashedOverlappingPairCache::default();

        let (cell_size_multiplier, initial_handle_size) = match config.mem_weight_mode {
            ArenaMemWeightMode::Light => (3.0, 1),
            ArenaMemWeightMode::Heavy => (1.0, 8),
        };

        let broadphase = GridBroadphase::new(
            config.min_pos * UU_TO_BT,
            config.max_pos * UU_TO_BT,
            config.max_aabb_len * UU_TO_BT * cell_size_multiplier,
            initial_handle_size,
            overlapping_pair_cache,
        );

        let mut bullet_world =
            DiscreteDynamicsWorld::new(collision_dispatcher, broadphase, constraint_solver);
        bullet_world.set_gravity(mutator_config.gravity * UU_TO_BT);

        if game_mode != GameMode::TheVoid {
            Self::setup_arena_collision_shapes(&mut bullet_world, game_mode);
        }

        let ball = Ball::new(
            game_mode,
            &mut bullet_world,
            &mutator_config,
            config.no_ball_rot,
        );

        let boost_pad_grid = {
            let mut boost_pad_configs = Vec::new();
            if game_mode != GameMode::TheVoid && game_mode != GameMode::Dropshot {
                if config.use_custom_boost_pads {
                    boost_pad_configs.extend_from_slice(&config.custom_boost_pads);
                } else {
                    let small_pad_locs = consts::boost_pads::get_locations(game_mode, false);
                    let big_pad_locs = consts::boost_pads::get_locations(game_mode, true);
                    boost_pad_configs.reserve(small_pad_locs.len() + big_pad_locs.len());

                    for small_pos in small_pad_locs {
                        boost_pad_configs.push(BoostPadConfig {
                            pos: *small_pos,
                            is_big: false,
                        });
                    }
                    for big_pos in big_pad_locs {
                        boost_pad_configs.push(BoostPadConfig {
                            pos: *big_pos,
                            is_big: true,
                        });
                    }
                }
            }

            BoostPadGrid::new(&boost_pad_configs)
        };

        let rng = config.rng_seed.map_or_else(Rng::new, Rng::with_seed);

        Self {
            rng,
            config,
            bullet_world,
            last_car_id: 0,
            tick_time: 1. / f32::from(tick_rate),
            data: ArenaInner {
                ball,
                game_mode,
                boost_pad_grid,
                mutator_config,
                tick_count: 0,
                cars: AHashMap::with_capacity(6),
            },
            goal_scored_callback: None,
            user_data: None,
        }
    }

    #[must_use]
    pub const fn get_config(&self) -> &ArenaConfig {
        &self.config
    }

    fn add_static_collision_shape(
        bullet_world: &mut DiscreteDynamicsWorld,
        shape: CollisionShapes,
        pos_bt: Vec3A,
        group: u8,
        mask: u8,
    ) {
        let mut rb_constrution_info = RigidBodyConstructionInfo::new(0.0, shape);
        rb_constrution_info.restitution = consts::arena::BASE_COEFS.restitution;
        rb_constrution_info.friction = consts::arena::BASE_COEFS.friction;
        rb_constrution_info.start_world_transform.translation = pos_bt;

        let shape_rb = RigidBody::new(rb_constrution_info);
        if (group | mask) != 0 {
            bullet_world.add_rigid_body(shape_rb, group, mask);
        } else {
            bullet_world.add_rigid_body_default(shape_rb);
        }
    }

    fn setup_arena_collision_shapes(bullet_world: &mut DiscreteDynamicsWorld, game_mode: GameMode) {
        debug_assert!(game_mode != GameMode::TheVoid);

        let collision_shapes = ARENA_COLLISION_SHAPES.read().unwrap();
        let collision_meshes = &collision_shapes
            .as_ref()
            .expect("Arena collision shapes are uninitialized - please call init(..) first.")
            [&game_mode];
        assert!(
            !collision_meshes.is_empty(),
            "No arena meshes found for the game mode {game_mode:?}"
        );

        for mesh in collision_meshes {
            let is_hoops_net = if game_mode == GameMode::Hoops {
                todo!()
            } else {
                false
            };

            let mask = if is_hoops_net {
                CollisionMasks::HoopsNet as u8
            } else {
                0
            };

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::TriangleMesh(mesh.clone()),
                Vec3A::ZERO,
                mask,
                mask,
            );
        }

        drop(collision_shapes);

        // TODO: Move to consts
        let (extent_x, floor, height) = match game_mode {
            GameMode::Hoops => (
                consts::arena::EXTENT_X_HOOPS,
                0.0,
                consts::arena::HEIGHT_HOOPS,
            ),
            GameMode::Dropshot => (
                consts::arena::EXTENT_X,
                consts::arena::FLOOR_HEIGHT_DROPSHOT,
                consts::arena::HEIGHT_DROPSHOT,
            ),
            _ => (consts::arena::EXTENT_X, 0.0, consts::arena::HEIGHT),
        };

        let mut add_plane = |pos_uu: Vec3A, normal: Vec3A, mask: u8| {
            debug_assert!(normal.is_normalized());
            let pos_bt = pos_uu * UU_TO_BT;
            let trans = Affine3A {
                matrix3: Mat3A::IDENTITY,
                translation: pos_bt,
            };

            let plane_shape = StaticPlaneShape::new(trans, normal);

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::StaticPlane(plane_shape),
                pos_bt,
                mask,
                mask,
            );
        };

        let floor_mask = if game_mode == GameMode::Dropshot {
            CollisionMasks::DropshotFloor as u8
        } else {
            0
        };

        // Floor
        add_plane(Vec3A::new(0.0, 0.0, floor), Vec3A::Z, floor_mask);

        // Ceiling
        add_plane(Vec3A::new(0.0, 0.0, height), Vec3A::NEG_Z, 0);

        match game_mode {
            GameMode::Hoops => {
                // Y walls
                add_plane(
                    Vec3A::new(0.0, -consts::arena::EXTENT_Y_HOOPS, height / 2.),
                    Vec3A::Y,
                    0,
                );

                add_plane(
                    Vec3A::new(0.0, consts::arena::EXTENT_Y_HOOPS, height / 2.),
                    Vec3A::NEG_Y,
                    0,
                );
            }
            GameMode::Dropshot => {
                // Add tiles
                todo!()
            }
            _ => {
                // Side walls
                add_plane(Vec3A::new(-extent_x, 0.0, height / 2.), Vec3A::X, 0);
                add_plane(Vec3A::new(extent_x, 0.0, height / 2.), Vec3A::NEG_X, 0);
            }
        }
    }

    fn ball_within_hoops_goal_xy_margin_eq(x: f32, y: f32) -> f32 {
        const SCALE_Y: f32 = 0.9;
        const OFFSET_Y: f32 = 2770.0;
        const RADIUS_SQ: f32 = 716.0 * 716.0;

        let dy = y.abs() * SCALE_Y - OFFSET_Y;
        let dist_sq = x * x + dy * dy;
        dist_sq - RADIUS_SQ
    }

    #[must_use]
    pub fn is_ball_scored(&self) -> bool {
        let ball_pos = self.bullet_world.bodies()[self.data.ball.rigid_body_idx]
            .collision_object
            .get_world_transform()
            .translation
            * BT_TO_UU;

        match self.data.game_mode {
            GameMode::Soccar | GameMode::Heatseeker | GameMode::Snowday => {
                ball_pos.y.abs()
                    > self.data.mutator_config.goal_base_threshold_y
                        + self.data.mutator_config.ball_radius
            }
            GameMode::Hoops => {
                if ball_pos.z < consts::goal::HOOPS_GOAL_SCORE_THRESHOLD_Z {
                    Self::ball_within_hoops_goal_xy_margin_eq(ball_pos.x, ball_pos.y) < 0.0
                } else {
                    false
                }
            }
            GameMode::Dropshot => ball_pos.z < -self.data.mutator_config.ball_radius * 1.75,
            GameMode::TheVoid => false,
        }
    }

    #[must_use]
    pub const fn get_tick_rate(&self) -> f32 {
        1.0 / self.tick_time
    }

    pub fn reset_to_random_kickoff(&mut self) {
        let game_mode = self.data.game_mode;
        let kickoff_locs = consts::car::spawn::get_kickoff_spawn_locations(game_mode);
        let respawn_locs = consts::car::spawn::get_respawn_locations(game_mode);

        let mut kickoff_order_perm = ArrayVec::<usize, 5>::new();
        kickoff_order_perm.extend(0..kickoff_locs.len());
        self.rng.shuffle(&mut kickoff_order_perm);

        let mut num_blue_cars = 0;
        let mut num_orange_cars = 0;

        for (_, car) in &mut self.data.cars {
            if car.team == Team::Blue {
                num_blue_cars += 1;
            } else {
                num_orange_cars += 1;
            }
        }

        let mut num_cars_at_respawn_pos = ArrayVec::<usize, 4>::new();
        num_cars_at_respawn_pos.extend(repeat_n(0, 4));

        let kickoff_position_amount = num_blue_cars.max(num_orange_cars);
        for i in 0..kickoff_position_amount {
            let spawn_pos = if i < kickoff_locs.len() {
                kickoff_locs[kickoff_order_perm[i]]
            } else {
                const CAR_SPAWN_EXTRA_OFFSET_Y: f32 = 250.0;

                let respawn_pos_idx = (i - kickoff_locs.len()) % respawn_locs.len();
                let mut pos = respawn_locs[respawn_pos_idx];
                pos.y += CAR_SPAWN_EXTRA_OFFSET_Y * num_cars_at_respawn_pos[respawn_pos_idx] as f32;
                num_cars_at_respawn_pos[respawn_pos_idx] += 1;

                pos
            };

            let mut spawn_state = CarState {
                phys: PhysState {
                    pos: Vec3A::new(spawn_pos.x, spawn_pos.y, consts::car::spawn::SPAWN_Z),
                    rot_mat: Mat3A::IDENTITY,
                    vel: Vec3A::ZERO,
                    ang_vel: Vec3A::ZERO,
                },
                boost: self.data.mutator_config.car_spawn_boost_amount,
                is_on_ground: true,
                ..Default::default()
            };

            for is_blue in [true, false] {
                let team_cars = self.data.cars.values_mut().filter(|car| {
                    if is_blue {
                        car.team == Team::Blue
                    } else {
                        car.team == Team::Orange
                    }
                });

                let Some(car) = team_cars.into_iter().nth(i) else {
                    continue;
                };

                spawn_state.phys.rot_mat = Mat3A::from_euler(
                    EulerRot::YZX,
                    0.0,
                    if is_blue {
                        spawn_pos.yaw_ang
                    } else {
                        spawn_state.phys.pos *= Vec3A::new(-1.0, -1.0, 1.0);
                        spawn_pos.yaw_ang + if is_blue { 0.0 } else { PI }
                    },
                    0.0,
                );

                car.set_state(
                    &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
                    &spawn_state,
                );
            }
        }

        let mut ball_state = BallState::DEFAULT;
        match self.data.game_mode {
            GameMode::Heatseeker => {
                let next_rand = self.rng.bool();
                let y_sign = f32::from(i8::from(next_rand) * 2 - 1);
                let scale = Vec3A::new(1.0, y_sign, 1.0);
                ball_state.phys.pos = consts::heatseeker::BALL_START_POS * scale;
                ball_state.phys.vel = consts::heatseeker::BALL_START_VEL * scale;
            }
            GameMode::Snowday => {
                ball_state.phys.vel.z = f32::EPSILON;
            }
            _ => {}
        }

        self.data.ball.set_state(
            &mut self.bullet_world.bodies_mut()[self.data.ball.rigid_body_idx],
            ball_state,
        );

        self.data.boost_pad_grid.reset();

        // TODO
        // Reset tile states
    }

    /// Adds a car to the match,
    /// returning the id of the car.
    /// The id is used as the key for the car in `Arena.cars`
    pub fn add_car(&mut self, team: Team, config: CarConfig) -> u64 {
        self.last_car_id += 1;
        let id = self.last_car_id;

        let mut car = Car::new(
            id,
            team,
            &mut self.bullet_world,
            &self.data.mutator_config,
            config,
        );
        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.data.game_mode,
            self.data.mutator_config.car_spawn_boost_amount,
        );

        self.bullet_world.bodies_mut()[car.rigid_body_idx]
            .collision_object
            .user_pointer = self.last_car_id;
        self.data.cars.insert(self.last_car_id, car);
        self.last_car_id
    }

    pub fn remove_car(&mut self, id: u64) -> bool {
        if let Some(car) = self.data.cars.remove(&id) {
            if car.rigid_body_idx < self.data.ball.rigid_body_idx {
                self.data.ball.rigid_body_idx -= 1;
            }

            for other_car in self.data.cars.values_mut() {
                if car.rigid_body_idx < other_car.rigid_body_idx {
                    other_car.rigid_body_idx -= 1;
                    other_car.bullet_vehicle.chassis_body_idx -= 1;
                }
            }

            self.bullet_world
                .remove_collision_object(car.rigid_body_idx);
            true
        } else {
            false
        }
    }

    pub fn remove_all_cars(&mut self) {
        while !self.cars().is_empty() {
            let id = *self.cars().keys().next().unwrap();
            self.remove_car(id);
        }
    }

    fn internal_step(&mut self) {
        {
            let ball_rb = &mut self.bullet_world.bodies_mut()[self.data.ball.rigid_body_idx];
            let should_sleep = ball_rb.linear_velocity.length_squared() == 0.0
                && ball_rb.angular_velocity.length_squared() == 0.0;

            ball_rb
                .collision_object
                .set_activation_state(if should_sleep {
                    ActivationState::Sleeping
                } else {
                    ActivationState::Active
                });
        }

        for car in self.data.cars.values_mut() {
            car.pre_tick_update(
                &mut self.bullet_world,
                &mut self.rng,
                self.data.game_mode,
                self.tick_time,
                &self.data.mutator_config,
            );
        }

        self.data
            .ball
            .pre_tick_update(self.data.game_mode, self.tick_time);

        self.bullet_world
            .step_simulation(self.tick_time, &mut self.data);

        for car in self.data.cars.values_mut() {
            let rb = &mut self.bullet_world.bodies_mut()[car.rigid_body_idx];
            car.post_tick_update(self.tick_time, rb);
            car.finish_physics_tick(rb);

            self.data.boost_pad_grid.maybe_give_car_boost(
                &mut car.internal_state,
                &self.data.mutator_config,
                self.data.tick_count,
                self.tick_time,
            );
        }

        self.data.ball.finish_physics_tick(
            &mut self.bullet_world.bodies_mut()[self.data.ball.rigid_body_idx],
            &self.data.mutator_config,
        );

        if self.data.game_mode == GameMode::Dropshot {
            todo!("dropshot tile state sync")
        }

        if let Some(callback) = self.goal_scored_callback
            && self.is_ball_scored()
        {
            let ball = &self.bullet_world.bodies()[self.data.ball.rigid_body_idx];
            let scoring_team = if ball.collision_object.get_world_transform().translation.y > 0.0 {
                Team::Blue
            } else {
                Team::Orange
            };

            callback(self, scoring_team);
        }

        self.data.tick_count += 1;
    }

    pub fn step(&mut self, ticks_to_simulate: u32) {
        for _ in 0..ticks_to_simulate {
            self.internal_step();
        }
    }

    #[inline]
    #[must_use]
    pub const fn tick_count(&self) -> u64 {
        self.data.tick_count
    }

    #[inline]
    #[must_use]
    pub const fn game_mode(&self) -> GameMode {
        self.data.game_mode
    }

    #[inline]
    #[must_use]
    pub const fn mutator_config(&self) -> &MutatorConfig {
        &self.data.mutator_config
    }

    #[inline]
    #[must_use]
    pub fn boost_pads(&self) -> &[BoostPad] {
        self.data.boost_pad_grid.pads()
    }

    #[inline]
    #[must_use]
    pub fn boost_pads_mut(&mut self) -> &mut [BoostPad] {
        self.data.boost_pad_grid.pads_mut()
    }

    #[inline]
    #[must_use]
    pub const fn get_ball(&self) -> &BallState {
        &self.data.ball.internal_state
    }

    #[inline]
    pub fn set_ball(&mut self, state: BallState) {
        self.data.ball.set_state(
            &mut self.bullet_world.bodies_mut()[self.data.ball.rigid_body_idx],
            state,
        );
    }

    #[inline]
    #[must_use]
    pub const fn cars(&self) -> &AHashMap<u64, Car> {
        &self.data.cars
    }

    #[must_use]
    pub fn car(&self, car_id: u64) -> &Car {
        self.data.cars.get(&car_id).unwrap()
    }

    #[must_use]
    pub fn car_mut(&mut self, car_id: u64) -> &mut Car {
        self.data.cars.get_mut(&car_id).unwrap()
    }

    #[must_use]
    pub fn get_car(&self, car_id: u64) -> Option<&Car> {
        self.data.cars.get(&car_id)
    }

    #[must_use]
    pub fn get_car_mut(&mut self, car_id: u64) -> Option<&mut Car> {
        self.data.cars.get_mut(&car_id)
    }

    pub fn set_car_state(&mut self, car_id: u64, state: CarState) {
        let car = self
            .data
            .cars
            .get_mut(&car_id)
            .expect("No car with the given id");

        car.set_state(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &state,
        );
    }

    pub fn respawn_car(&mut self, car_id: u64) {
        let car = self
            .data
            .cars
            .get_mut(&car_id)
            .expect("No car with the given id");

        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.data.game_mode,
            self.data.mutator_config.car_spawn_boost_amount,
        );
    }

    #[must_use]
    pub fn get_game_state(&self) -> GameState {
        GameState {
            tick_rate: self.get_tick_rate(),
            tick_count: self.tick_count(),
            game_mode: self.game_mode(),
            cars: if self.cars().is_empty() {
                None
            } else {
                Some(
                    self.cars()
                        .iter()
                        .map(|(&id, car)| CarInfo {
                            id,
                            team: car.team,
                            state: *car.get_state(),
                            config: *car.get_config(),
                        })
                        .collect(),
                )
            },
            ball: *self.get_ball(),
            pads: if self.boost_pads().is_empty() {
                None
            } else {
                Some(
                    self.boost_pads()
                        .iter()
                        .map(|pad| BoostPadInfo {
                            config: *pad.config(),
                            state: pad.internal_state,
                        })
                        .collect(),
                )
            },
            tiles: None,
        }
    }

    pub fn set_game_state(&mut self, state: GameState) {
        assert_eq!(self.game_mode(), state.game_mode, "Game mode mismatch");

        if let Some(cars) = state.cars {
            if cars.len() != self.cars().len() {
                panic!(
                    "Car count mismatch: expected {}, got {}",
                    self.cars().len(),
                    cars.len()
                );
            }

            for car_info in cars {
                self.set_car_state(car_info.id, car_info.state);
            }
        }

        self.set_ball(state.ball);

        if let Some(pads) = state.pads {
            if pads.len() != self.boost_pads().len() {
                panic!(
                    "Boost pad count mismatch: expected {}, got {}",
                    self.boost_pads().len(),
                    pads.len()
                );
            }

            for i in 0..self.boost_pads().len() {
                self.set_boost_pad_state(i, pads[i].state);
            }
        }

        // todo: tiles
    }
}
