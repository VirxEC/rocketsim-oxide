use super::{Ball, BoostPadConfig, Car, CarConfig, CarState, MutatorConfig, PhysState, Team};
use crate::{
    ARENA_COLLISION_SHAPES, BT_TO_UU, GameMode, UU_TO_BT, UserInfoTypes,
    bullet::{
        collision::{
            broadphase::{
                overlapping_pair_cache::HashedOverlappingPairCache, rs_broadphase::RsBroadphase,
            },
            dispatch::{
                collision_dispatcher::CollisionDispatcher,
                collision_object::{ACTIVE_TAG, CollisionObject, ISLAND_SLEEPING},
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
    consts::{
        heatseeker::{BALL_START_POS, BALL_START_VEL},
        *,
    },
    sim::{BallState, BoostPad, CarContact, CollisionMasks, DemoMode, mutator_config},
};
use ahash::AHashMap;
use fastrand::Rng;
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};
use std::{array::from_fn, cell::RefCell, f32::consts::PI, mem, rc::Rc};

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum ArenaMemWeightMode {
    #[default]
    Heavy,
    Light,
}

#[derive(Clone, Debug)]
pub struct ArenaConfig {
    pub mem_weight_mode: ArenaMemWeightMode,
    pub min_pos: Vec3A,
    pub max_pos: Vec3A,
    pub max_aabb_len: f32,
    pub no_ball_rot: bool,
    /// Use a custom list of boost pads (`custom_boost_pads`) instead of the normal one
    /// NOTE: This will disable the boost pad grid and will thus worsen performance
    pub use_custom_boost_pads: bool,
    /// Custom boost pads to use, if `use_custom_boost_pads`
    pub custom_boost_pads: Vec<BoostPadConfig>,
    /// Optional RNG seed for deterministic behavior
    /// If None, a random seed will be used
    pub rng_seed: Option<u64>,
}

impl Default for ArenaConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ArenaConfig {
    pub const DEFAULT: Self = Self {
        mem_weight_mode: ArenaMemWeightMode::Heavy,
        min_pos: Vec3A::new(-5600., -6000., 0.),
        max_pos: Vec3A::new(5600., 6000., 2200.),
        max_aabb_len: 370.,
        no_ball_rot: true,
        use_custom_boost_pads: false,
        custom_boost_pads: Vec::new(),
        rng_seed: None,
    };
}

pub struct Objects {
    pub ball: Ball,
    /// Do NOT add/remove cars by adding/removing them from the hashmap.
    pub cars: AHashMap<u64, Car>,
    tick_count: u64,
    game_mode: GameMode,
    mutator_config: MutatorConfig,
    boost_pads: Vec<BoostPad>,
}

impl Objects {
    fn on_car_ball_collision(
        &mut self,
        car_id: u64,
        manifold_point: &mut ManifoldPoint,
        ball_is_body_a: bool,
    ) {
        manifold_point.combined_friction = CARBALL_COLLISION_FRICTION;
        manifold_point.combined_restitution = CARBALL_COLLISION_RESTITUTION;

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
        car_1_id: u64,
        car_2_id: u64,
        manifold_point: &mut ManifoldPoint,
    ) {
        manifold_point.combined_friction = CARCAR_COLLISION_FRICTION;
        manifold_point.combined_restitution = CARCAR_COLLISION_RESTITUTION;

        // SAFETY: car_1_id and car_2_id are guaranteed to be different;
        // a car cannot collide with itself.
        let [car_1, car_2] =
            unsafe { self.cars.get_disjoint_unchecked_mut([&car_1_id, &car_2_id]) };
        let car_1 = car_1.unwrap();
        let car_2 = car_2.unwrap();

        let mut state_1 = car_1.get_state();
        let mut state_2 = car_2.get_state();

        if state_1.is_demoed || state_2.is_demoed {
            return;
        }

        // Test collision both ways
        for is_swapped in [false, true] {
            if is_swapped {
                mem::swap(car_1, car_2);
                mem::swap(&mut state_1, &mut state_2);
            }

            if let Some(car_contact) = state_1.car_contact
                && car_contact.other_car_id == car_2_id
                && car_contact.cooldown_timer > 0.0
            {
                // In cooldown
                continue;
            }

            let delta_pos = state_2.physics.pos - state_1.physics.pos;
            if state_1.physics.vel.dot(delta_pos) < 0.0 {
                // Moving away from the other car
                continue;
            }

            let vel_dir = state_1.physics.vel.normalize();
            let dir_to_other_car = delta_pos.normalize();

            let speed_towards_other_car = state_1.physics.vel.dot(dir_to_other_car);
            let other_car_away_speed = state_2.physics.vel.dot(vel_dir);
            if speed_towards_other_car <= other_car_away_speed {
                // Going towards other car slower than they're going away
                continue;
            }

            let local_point = if is_swapped {
                manifold_point.local_point_b
            } else {
                manifold_point.local_point_a
            };

            let hit_with_bumper = local_point.x * BT_TO_UU > BUMP_MIN_FORWARD_DIST;
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
                    BUMP_VEL_AMOUNT_GROUND_CURVE
                } else {
                    BUMP_VEL_AMOUNT_AIR_CURVE
                }
                .get_output(speed_towards_other_car);

                let hit_up_dir = if state_2.is_on_ground {
                    state_2.physics.rot_mat.z_axis
                } else {
                    Vec3A::Z
                };

                let bump_impulse = vel_dir * base_scale
                    + hit_up_dir
                        * BUMP_UPWARD_VEL_AMOUNT_CURVE.get_output(speed_towards_other_car)
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

impl ContactAddedCallback for Objects {
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
                    self.on_car_ball_collision(body_a.user_pointer, contact_point, should_swap)
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

pub struct Arena {
    rng: Rng,
    tick_time: f32,
    last_car_id: u64,
    config: ArenaConfig,
    bullet_world: DiscreteDynamicsWorld,
    pub objects: Objects,
}

impl Arena {
    #[must_use]
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(game_mode, ArenaConfig::DEFAULT, 120)
    }

    pub fn new_with_config(game_mode: GameMode, config: ArenaConfig, tick_rate: u8) -> Self {
        assert!(tick_rate >= 15, "tick_rate must be at least 15.0");
        assert!(tick_rate <= 120, "tick_rate must not be greater than 120.0");

        let mutator_config = MutatorConfig::new(game_mode);

        let collision_dispatcher = CollisionDispatcher::default();
        let constraint_solver = SequentialImpulseConstraintSolver::default();
        let overlapping_pair_cache = HashedOverlappingPairCache::default();

        let cell_size_multiplier = match config.mem_weight_mode {
            ArenaMemWeightMode::Light => 2.0,
            ArenaMemWeightMode::Heavy => 1.0,
        };

        let broadphase = RsBroadphase::new(
            config.min_pos * UU_TO_BT,
            config.max_pos * UU_TO_BT,
            config.max_aabb_len * UU_TO_BT * cell_size_multiplier,
            overlapping_pair_cache,
        );

        let mut bullet_world =
            DiscreteDynamicsWorld::new(collision_dispatcher, broadphase, constraint_solver);
        bullet_world.set_gravity(mutator_config.gravity * UU_TO_BT);

        let solver_info = &mut bullet_world.dynamics_world.solver_info;
        solver_info.split_impulse_penetration_threshold = 1e30;
        solver_info.erp_2 = 0.8;

        if game_mode != GameMode::TheVoid {
            Self::setup_arena_collision_shapes(&mut bullet_world, game_mode);
        }

        let ball = Ball::new(
            game_mode,
            &mut bullet_world,
            &mutator_config,
            config.no_ball_rot,
        );

        let mut boost_pads = Vec::new();

        if game_mode != GameMode::TheVoid && game_mode != GameMode::Dropshot {
            if config.use_custom_boost_pads {
                boost_pads.reserve(config.custom_boost_pads.len());
                boost_pads.extend(config.custom_boost_pads.iter().copied().map(BoostPad::new));
            } else {
                let amount_small = if game_mode == GameMode::Hoops {
                    boostpads::LOCS_AMOUNT_SMALL_HOOPS
                } else {
                    boostpads::LOCS_AMOUNT_SMALL_SOCCAR
                };

                let num_pads = boostpads::LOCS_AMOUNT_BIG + amount_small;
                boost_pads.reserve(num_pads);
                for i in 0..num_pads {
                    let is_big = i < boostpads::LOCS_AMOUNT_BIG;

                    let pos = if game_mode == GameMode::Hoops {
                        if is_big {
                            boostpads::LOCS_BIG_HOOPS[i]
                        } else {
                            boostpads::LOCS_SMALL_HOOPS[i - boostpads::LOCS_AMOUNT_BIG]
                        }
                    } else if is_big {
                        boostpads::LOCS_BIG_SOCCAR[i]
                    } else {
                        boostpads::LOCS_SMALL_SOCCAR[i - boostpads::LOCS_AMOUNT_BIG]
                    };

                    let pad_config = BoostPadConfig { pos, is_big };

                    boost_pads.push(BoostPad::new(pad_config));
                }
            }
        }

        let rng = match config.rng_seed {
            Some(seed) => Rng::with_seed(seed),
            None => Rng::new(),
        };

        Self {
            rng,
            config,
            bullet_world,
            last_car_id: 0,
            tick_time: 1. / f32::from(tick_rate),
            objects: Objects {
                ball,
                game_mode,
                boost_pads,
                mutator_config,
                tick_count: 0,
                cars: AHashMap::new(),
            },
        }
    }

    fn add_static_collision_shape(
        bullet_world: &mut DiscreteDynamicsWorld,
        shape: CollisionShapes,
        pos_bt: Vec3A,
        group: i32,
        mask: i32,
    ) {
        let mut rb_constrution_info = RigidBodyConstructionInfo::new(0.0, shape);
        rb_constrution_info.restitution = ARENA_COLLISION_BASE_RESTITUTION;
        rb_constrution_info.friction = ARENA_COLLISION_BASE_FRICTION;
        rb_constrution_info.start_world_transform.translation = pos_bt;

        let shape_rb = Rc::new(RefCell::new(RigidBody::new(rb_constrution_info)));
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
                CollisionMasks::HoopsNet as i32
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

        let (extent_x, floor, height) = match game_mode {
            GameMode::Hoops => (ARENA_EXTENT_X_HOOPS, 0.0, ARENA_HEIGHT_HOOPS),
            GameMode::Dropshot => (ARENA_EXTENT_X, FLOOR_HEIGHT_DROPSHOT, ARENA_HEIGHT_DROPSHOT),
            _ => (ARENA_EXTENT_X, 0.0, ARENA_HEIGHT),
        };

        let mut add_plane = |pos_uu: Vec3A, normal: Vec3A, mask: i32| {
            debug_assert!(normal.is_normalized());
            let mut plane_shape = StaticPlaneShape::new(normal, 0.0);

            let pos_bt = pos_uu * UU_TO_BT;
            let trans = Affine3A {
                matrix3: Mat3A::IDENTITY,
                translation: pos_bt,
            };
            let aabb = plane_shape.get_aabb(&trans);
            plane_shape.concave_shape.collision_shape.aabb_cache = Some(aabb);
            plane_shape.concave_shape.collision_shape.aabb_cache_trans = trans;
            plane_shape.concave_shape.collision_shape.aabb_ident_cache =
                Some(plane_shape.get_aabb(&Affine3A::IDENTITY));

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::StaticPlane(plane_shape),
                pos_bt,
                mask,
                mask,
            );
        };

        let floor_mask = if game_mode == GameMode::Dropshot {
            CollisionMasks::DropshotFloor as i32
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
                    Vec3A::new(0.0, -ARENA_EXTENT_Y_HOOPS, height / 2.),
                    Vec3A::Y,
                    0,
                );

                add_plane(
                    Vec3A::new(0.0, ARENA_EXTENT_Y_HOOPS, height / 2.),
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
        let ball_pos = self
            .objects
            .ball
            .rigid_body
            .borrow()
            .collision_object
            .borrow()
            .get_world_transform()
            .translation
            * BT_TO_UU;

        match self.objects.game_mode {
            GameMode::Soccar | GameMode::Heatseeker | GameMode::Snowday => {
                ball_pos.y.abs()
                    > self.objects.mutator_config.goal_base_threshold_y
                        + self.objects.mutator_config.ball_radius
            }
            GameMode::Hoops => {
                if ball_pos.z < HOOPS_GOAL_SCORE_THRESHOLD_Z {
                    Self::ball_within_hoops_goal_xy_margin_eq(ball_pos.x, ball_pos.y) < 0.0
                } else {
                    false
                }
            }
            GameMode::Dropshot => ball_pos.z < -self.objects.mutator_config.ball_radius * 1.75,
            GameMode::TheVoid => false,
        }
    }

    #[must_use]
    pub const fn get_tick_rate(&self) -> f32 {
        1.0 / self.tick_time
    }

    pub fn reset_to_random_kickoff(&mut self) {
        let mut kickoff_order: [usize; CAR_SPAWN_LOCATION_AMOUNT] = from_fn(|i| i);
        self.rng.shuffle(&mut kickoff_order);

        let (location_amount, car_spawn_locations, car_respawn_locations) =
            match self.objects.game_mode {
                GameMode::Hoops => (
                    CAR_SPAWN_LOCATION_AMOUNT,
                    CAR_SPAWN_LOCATIONS_HOOPS.as_slice(),
                    CAR_RESPAWN_LOCATIONS_HOOPS,
                ),
                GameMode::Heatseeker => (
                    CAR_SPAWN_LOCATION_AMOUNT_HEATSEEKER,
                    CAR_SPAWN_LOCATIONS_HEATSEEKER.as_slice(),
                    CAR_RESPAWN_LOCATIONS_SOCCAR,
                ),
                GameMode::Dropshot => (
                    CAR_SPAWN_LOCATION_AMOUNT,
                    CAR_SPAWN_LOCATIONS_DROPSHOT.as_slice(),
                    CAR_RESPAWN_LOCATIONS_DROPSHOT,
                ),
                _ => (
                    CAR_SPAWN_LOCATION_AMOUNT,
                    CAR_SPAWN_LOCATIONS_SOCCAR.as_slice(),
                    CAR_RESPAWN_LOCATIONS_SOCCAR,
                ),
            };

        let mut blue_cars = Vec::with_capacity(self.objects.cars.len().div_ceil(2));
        let mut orange_cars = Vec::with_capacity(self.objects.cars.len().div_ceil(2));

        for (_, car) in &mut self.objects.cars {
            if car.team == Team::Blue {
                &mut blue_cars
            } else {
                &mut orange_cars
            }
            .push(car);
        }

        let mut num_cars_at_respawn_pos = [0; CAR_RESPAWN_LOCATION_AMOUNT];

        let kickoff_position_amount = blue_cars.len().max(orange_cars.len());
        for i in 0..kickoff_position_amount {
            let spawn_pos = if i < location_amount {
                car_spawn_locations[kickoff_order[i].min(location_amount - 1)]
            } else {
                const CAR_SPAWN_EXTRA_OFFSET_Y: f32 = 250.0;

                let respawn_pos_idx = (i - location_amount) % location_amount;
                let mut pos = car_respawn_locations[respawn_pos_idx];
                pos.y += CAR_SPAWN_EXTRA_OFFSET_Y * num_cars_at_respawn_pos[respawn_pos_idx] as f32;
                num_cars_at_respawn_pos[respawn_pos_idx] += 1;

                pos
            };

            let mut spawn_state = CarState {
                physics: PhysState {
                    pos: Vec3A::new(spawn_pos.x, spawn_pos.y, CAR_SPAWN_REST_Z),
                    rot_mat: Mat3A::IDENTITY,
                    vel: Vec3A::ZERO,
                    ang_vel: Vec3A::ZERO,
                },
                boost: self.objects.mutator_config.car_spawn_boost_amount,
                is_on_ground: true,
                ..Default::default()
            };

            for is_blue in [true, false] {
                let team_cars = if is_blue {
                    blue_cars.as_mut_slice()
                } else {
                    orange_cars.as_mut_slice()
                };

                let Some(car) = team_cars.get_mut(i) else {
                    continue;
                };

                spawn_state.physics.rot_mat = Mat3A::from_euler(
                    EulerRot::YZX,
                    0.0,
                    if is_blue {
                        spawn_pos.yaw_ang
                    } else {
                        spawn_state.physics.pos *= Vec3A::new(-1.0, -1.0, 1.0);
                        spawn_pos.yaw_ang + if is_blue { 0.0 } else { PI }
                    },
                    0.0,
                );

                car.set_state(spawn_state);
            }
        }

        let mut ball_state = BallState::DEFAULT;
        match self.objects.game_mode {
            GameMode::Heatseeker => {
                let next_rand = self.rng.bool();
                let scale = Vec3A::new(1.0, f32::from(i8::from(next_rand) * 2 - 1), 1.0);
                ball_state.physics.pos = BALL_START_POS * scale;
                ball_state.physics.vel = BALL_START_VEL * scale;
            }
            GameMode::Snowday => {
                ball_state.physics.vel.z = f32::EPSILON;
            }
            _ => {}
        }
        self.objects.ball.set_state(ball_state);

        // TODO
        // Reset boost pads
        // Reset tile states
    }

    /// Adds a car to the match,
    /// returning the id of the car.
    /// The id is used as the key for the car in `Arena.cars`
    pub fn add_car(&mut self, team: Team, config: CarConfig) -> u64 {
        let mut car = Car::new(
            &mut self.bullet_world,
            &self.objects.mutator_config,
            team,
            config,
        );
        car.respawn(
            self.objects.game_mode,
            self.objects.mutator_config.car_spawn_boost_amount,
        );

        self.last_car_id += 1;
        car.rigid_body
            .borrow()
            .collision_object
            .borrow_mut()
            .user_pointer = self.last_car_id;
        self.objects.cars.insert(self.last_car_id, car);
        self.last_car_id
    }

    fn internal_step(&mut self) {
        {
            let ball_rb = self.objects.ball.rigid_body.borrow();
            ball_rb.collision_object.borrow_mut().set_activation_state(
                if ball_rb.linear_velocity.length_squared() == 0.0
                    && ball_rb.angular_velocity.length_squared() == 0.0
                {
                    ISLAND_SLEEPING
                } else {
                    ACTIVE_TAG
                },
            );
        }

        for car in self.objects.cars.values_mut() {
            car.pre_tick_update(
                &self.bullet_world,
                self.objects.game_mode,
                self.tick_time,
                &self.objects.mutator_config,
            );
        }

        let ball_only = self.objects.cars.is_empty();
        let has_arena_stuff = self.objects.game_mode != GameMode::TheVoid;

        if has_arena_stuff && !ball_only {
            // todo: boostpad pretickupdate
        }

        self.objects
            .ball
            .pre_tick_update(self.objects.game_mode, self.tick_time);

        self.bullet_world
            .step_simulation(self.tick_time, &mut self.objects);

        for car in self.objects.cars.values_mut() {
            car.post_tick_update(self.tick_time);
            car.finish_physics_tick();

            if has_arena_stuff {
                // todo: boostpad collision checks
            }
        }

        if has_arena_stuff && !ball_only {
            // todo: boostpad posttickupdate
        }

        self.objects
            .ball
            .finish_physics_tick(&self.objects.mutator_config);

        if self.objects.game_mode == GameMode::Dropshot {
            todo!("dropshot tile state sync")
        }

        // todo: goalscorecallback

        self.objects.tick_count += 1;
    }

    pub fn step(&mut self, ticks_to_simulate: u32) {
        for i in 0..ticks_to_simulate {
            // println!("i: {i}");
            self.internal_step();
        }
    }

    #[inline]
    pub fn tick_count(&self) -> u64 {
        self.objects.tick_count
    }

    #[inline]
    pub fn game_mode(&self) -> GameMode {
        self.objects.game_mode
    }

    #[inline]
    pub fn mutator_config(&self) -> MutatorConfig {
        self.objects.mutator_config
    }

    #[inline]
    pub fn boost_pads(&self) -> &[BoostPad] {
        &self.objects.boost_pads
    }
}
