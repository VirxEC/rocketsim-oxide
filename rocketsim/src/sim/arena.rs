use super::{Ball, BoostPadConfig, MutatorConfig};
use crate::{
    ARENA_COLLISION_SHAPES, GameMode, UU_TO_BT, consts,
    sim::{BoostPad, CollisionMasks},
};
use bullet::{
    collision::{
        broadphase::{
            overlapping_pair_cache::HashedOverlappingPairCache, rs_broadphase::RsBroadphase,
        },
        dispatch::{
            collision_dispatcher::CollisionDispatcher,
            collision_object::{ACTIVE_TAG, ISLAND_SLEEPING},
        },
        shapes::{collision_shape::CollisionShapes, static_plane_shape::StaticPlaneShape},
    },
    dynamics::{
        constraint_solver::sequential_impulse_constraint_solver::SequentialImpulseConstraintSolver,
        discrete_dynamics_world::DiscreteDynamicsWorld,
        rigid_body::{RigidBody, RigidBodyConstructionInfo},
    },
};
use glam::{Affine3A, Mat3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum ArenaMemWeightMode {
    #[default]
    Heavy,
    Light,
}

#[derive(Clone, Debug)]
pub struct ArenaConfig {
    mem_weight_mode: ArenaMemWeightMode,
    min_pos: Vec3A,
    max_pos: Vec3A,
    max_aabb_len: f32,
    no_ball_rot: bool,
    use_custom_broadphase: bool,
    max_objects: usize,
    /// Use a custom list of boost pads (`custom_boost_pads`) instead of the normal one
    /// NOTE: This will disable the boost pad grid and will thus worsen performance
    use_custom_boost_pads: bool,
    /// Custom boost pads to use, if `use_custom_boost_pads`
    custom_boost_pads: Vec<BoostPadConfig>,
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
        use_custom_broadphase: true,
        max_objects: 512,
        use_custom_boost_pads: false,
        custom_boost_pads: Vec::new(),
    };
}

pub struct Arena {
    game_mode: GameMode,
    arena_config: ArenaConfig,
    mutator_config: MutatorConfig,
    tick_time: f32,
    bullet_world: DiscreteDynamicsWorld,
    pub ball: Ball,
    boost_pads: Vec<BoostPad>,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(game_mode, ArenaConfig::DEFAULT, 120.)
    }

    pub fn new_with_config(game_mode: GameMode, config: ArenaConfig, tick_rate: f32) -> Self {
        assert!(tick_rate >= 15.0, "tick_rate must be at least 15.0");
        assert!(
            tick_rate <= 120.0,
            "tick_rate must not be greater than 120.0"
        );

        let mutator_config = MutatorConfig::new(game_mode);

        let collision_dispatcher = CollisionDispatcher::default();
        let constraint_solver = SequentialImpulseConstraintSolver::default();
        let overlapping_pair_cache = Box::new(HashedOverlappingPairCache::default());

        let broadphase = if config.use_custom_broadphase {
            let cell_size_multiplier = match config.mem_weight_mode {
                ArenaMemWeightMode::Light => 2.0,
                ArenaMemWeightMode::Heavy => 1.0,
            };

            Box::new(RsBroadphase::new(
                config.min_pos * UU_TO_BT,
                config.max_pos * UU_TO_BT,
                config.max_aabb_len * UU_TO_BT * cell_size_multiplier,
                overlapping_pair_cache,
                config.max_objects,
            ))
        } else {
            unimplemented!();
        };

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
                    consts::boostpads::LOCS_AMOUNT_SMALL_HOOPS
                } else {
                    consts::boostpads::LOCS_AMOUNT_SMALL_SOCCAR
                };

                let num_pads = consts::boostpads::LOCS_AMOUNT_BIG + amount_small;
                boost_pads.reserve(num_pads);
                for i in 0..num_pads {
                    let is_big = i < consts::boostpads::LOCS_AMOUNT_BIG;

                    let pos = if game_mode == GameMode::Hoops {
                        if is_big {
                            consts::boostpads::LOCS_BIG_HOOPS[i]
                        } else {
                            consts::boostpads::LOCS_SMALL_HOOPS
                                [i - consts::boostpads::LOCS_AMOUNT_BIG]
                        }
                    } else if is_big {
                        consts::boostpads::LOCS_BIG_SOCCAR[i]
                    } else {
                        consts::boostpads::LOCS_SMALL_SOCCAR[i - consts::boostpads::LOCS_AMOUNT_BIG]
                    };

                    let pad_config = BoostPadConfig { pos, is_big };

                    boost_pads.push(BoostPad::new(pad_config));
                }
            }
        }

        Self {
            game_mode,
            tick_time: 1. / tick_rate,
            arena_config: config,
            mutator_config,
            bullet_world,
            ball,
            boost_pads,
        }
    }

    fn add_static_collision_shape(
        bullet_world: &mut DiscreteDynamicsWorld,
        shape: CollisionShapes,
        pos_bt: Vec3A,
        group: i32,
        mask: i32,
    ) {
        let mut rb_constrution_info = RigidBodyConstructionInfo::new(0.0, None, shape);
        rb_constrution_info.restitution = consts::ARENA_COLLISION_BASE_RESTITUTION;
        rb_constrution_info.friction = consts::ARENA_COLLISION_BASE_FRICTION;
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
                CollisionShapes::TriangleMesh(mesh.triangle_mesh_shape.clone()),
                Vec3A::ZERO,
                mask,
                mask,
            )
        }

        let (extent_x, floor, height) = match game_mode {
            GameMode::Hoops => (
                consts::ARENA_EXTENT_X_HOOPS,
                0.0,
                consts::ARENA_HEIGHT_HOOPS,
            ),
            GameMode::Dropshot => (
                consts::ARENA_EXTENT_X,
                consts::FLOOR_HEIGHT_DROPSHOT,
                consts::ARENA_HEIGHT_DROPSHOT,
            ),
            _ => (consts::ARENA_EXTENT_X, 0.0, consts::ARENA_HEIGHT),
        };

        let mut add_plane = |pos_uu: Vec3A, normal: Vec3A, mask: i32| {
            debug_assert!(normal.is_normalized());
            let mut plane_shape = StaticPlaneShape::new(normal, 0.0);

            let pos_bt = pos_uu * UU_TO_BT;
            let trans = Affine3A {
                matrix3: Mat3A::IDENTITY,
                translation: pos_bt,
            };
            let (aabb_min, aabb_max) = plane_shape.get_aabb(&trans);
            plane_shape.concave_shape.collision_shape.aabb_cached = true;
            plane_shape.concave_shape.collision_shape.aabb_min_cache = aabb_min;
            plane_shape.concave_shape.collision_shape.aabb_max_cache = aabb_max;
            plane_shape.concave_shape.collision_shape.aabb_cache_trans = trans;

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
        add_plane(Vec3A::new(0.0, 0.0, height), Vec3A::Z, 0);

        match game_mode {
            GameMode::Hoops => {
                // Y walls
                add_plane(
                    Vec3A::new(0.0, -consts::ARENA_EXTENT_Y_HOOPS, height / 2.),
                    Vec3A::Y,
                    0,
                );

                add_plane(
                    Vec3A::new(0.0, consts::ARENA_EXTENT_Y_HOOPS, height / 2.),
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

    fn internal_step(&mut self) {
        let ball_rb = self.ball.rigid_body.borrow();
        ball_rb.collision_object.borrow_mut().set_activation_state(
            if ball_rb.linear_velocity.length_squared() == 0.0
                && ball_rb.angular_velocity.length_squared() == 0.0
            {
                ISLAND_SLEEPING
            } else {
                ACTIVE_TAG
            },
        );
        drop(ball_rb);

        let ball_only = true;
        let has_arena_stuff = self.game_mode != GameMode::TheVoid;

        if has_arena_stuff && !ball_only {
            todo!()
        }

        self.ball.pre_tick_update(self.game_mode, self.tick_time);

        self.bullet_world
            .step_simulation(self.tick_time, 0, self.tick_time);
    }

    pub fn step(&mut self, ticks_to_simulate: u32) {
        for _ in 0..ticks_to_simulate {
            self.internal_step();
        }
    }
}
