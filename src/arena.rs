use std::{cell::RefCell, rc::Rc, sync::RwLock};

use crate::{
    assert_initialized,
    bullet::btdynamics::dynamics::{
        discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody,
    },
    bullet_link::UU_TO_BT,
    mutator_config::MutatorConfig,
    ARENA_COLLISION_MESH,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GameMode {
    #[default]
    Soccer,
    Hoops,
    HeatSeeker,
    SnowDay,
    TheVoid,
}

pub struct Arena {
    game_mode: GameMode,
    tick_time: f32,
    mutator_config: MutatorConfig,
    bullet_world: DiscreteDynamicsWorld,
    world_collision_rb: Rc<RefCell<RigidBody>>,
}

impl Default for Arena {
    fn default() -> Self {
        Self::create(GameMode::default(), 120)
    }
}

impl Arena {
    fn setup_arena_collision_shapes(&mut self) {
        assert_ne!(self.game_mode, GameMode::TheVoid);

        self.world_collision_rb
            .borrow_mut()
            .collision_object
            .set_collision_shape(ARENA_COLLISION_MESH.read().unwrap().clone());
        self.bullet_world
            .add_rigid_body(self.world_collision_rb.clone());

        // _worldCollisionBvhShapes = new btBvhTriangleMeshShape[collisionMeshes.size()];

        // size_t planeAmount = isHoops ? 6 : 4;
        // _worldCollisionPlaneShapes = new btStaticPlaneShape[planeAmount];

        // _worldCollisionRBAmount = collisionMeshes.size() + planeAmount;
        // _worldCollisionRBs = new btRigidBody[_worldCollisionRBAmount];

        // for (size_t i = 0; i < collisionMeshes.size(); i++) {
        //     auto mesh = collisionMeshes[i];

        //     bool isHoopsNet = false;

        //     if (isHoops) { // Detect net mesh and disable car collision
        //         const unsigned char* vertexBase;
        //         int numVerts, stride;
        //         const unsigned char* indexBase;
        //         int indexStride, numFaces;
        //         mesh->getMeshInterface()->getLockedReadOnlyVertexIndexBase(&vertexBase, numVerts, stride, &indexBase, indexStride, numFaces);

        //         constexpr int HOOPS_NET_NUM_VERTS = 505;
        //         if (numVerts == HOOPS_NET_NUM_VERTS) {
        //             isHoopsNet = true;
        //         }
        //     }

        //     _AddStaticCollisionShape(i, i, mesh, _worldCollisionBvhShapes, btVector3(0,0,0), isHoopsNet);

        //     // Don't free the BVH when we deconstruct this arena
        //     _worldCollisionBvhShapes[i].m_ownsBvh = false;
        // }

        // { // Add arena collision planes (floor/walls/ceiling)
        //     using namespace RLConst;

        //     float
        //         extentX = isHoops ? ARENA_EXTENT_X_HOOPS : ARENA_EXTENT_X,
        //         extentY = isHoops ? ARENA_EXTENT_Y_HOOPS : ARENA_EXTENT_Y,
        //         height  = isHoops ? ARENA_HEIGHT_HOOPS : ARENA_HEIGHT;

        //     // TODO: This is all very repetitive and silly

        //     // Floor
        //     auto floorShape = btStaticPlaneShape(btVector3(0, 0, 1), 0);
        //     _AddStaticCollisionShape(
        //         collisionMeshes.size() + 0,
        //         0,
        //         &floorShape, _worldCollisionPlaneShapes
        //     );

        //     // Ceiling
        //     auto ceilingShape = btStaticPlaneShape(btVector3(0, 0, -1), 0);
        //     _AddStaticCollisionShape(
        //         collisionMeshes.size() + 1,
        //         1,
        //         &ceilingShape, _worldCollisionPlaneShapes,
        //         Vec(0, 0, height) * UU_TO_BT
        //     );

        //     // Side walls
        //     auto leftWallShape = btStaticPlaneShape(btVector3(1, 0, 0), 0);
        //     _AddStaticCollisionShape(
        //         collisionMeshes.size() + 2,
        //         2,
        //         &leftWallShape, _worldCollisionPlaneShapes,
        //         Vec(-extentX, 0, height / 2) * UU_TO_BT
        //     );
        //     auto rightWallShape = btStaticPlaneShape(btVector3(-1, 0, 0), 0);
        //     _AddStaticCollisionShape(
        //         collisionMeshes.size() + 3,
        //         3,
        //         &rightWallShape, _worldCollisionPlaneShapes,
        //         Vec(extentX, 0, height / 2) * UU_TO_BT
        //     );

        //     if (isHoops) {
        //         // Y walls
        //         auto blueWallShape = btStaticPlaneShape(btVector3(0, 1, 0), 0);
        //         _AddStaticCollisionShape(
        //             collisionMeshes.size() + 4,
        //             4,
        //             &blueWallShape, _worldCollisionPlaneShapes,
        //             Vec(0, -extentY, height / 2) * UU_TO_BT
        //         );
        //         auto orangeWallShape = btStaticPlaneShape(btVector3(0, -1, 0), 0);
        //         _AddStaticCollisionShape(
        //             collisionMeshes.size() + 5,
        //             5,
        //             &orangeWallShape, _worldCollisionPlaneShapes,
        //             Vec(0, extentY, height / 2) * UU_TO_BT
        //         );
        //     }
        // }
    }

    /// Creates a new Arena with the given GameMode and tick rate.
    ///
    /// Tick rate must be in the range 15..=120.
    pub fn create(game_mode: GameMode, tick_rate: u8) -> Self {
        assert!((15..=120).contains(&tick_rate));
        assert_initialized("Cannot create Arena, ");

        let mut arena = Self {
            game_mode,
            tick_time: 1. / f32::from(tick_rate),
            mutator_config: MutatorConfig::new(game_mode),
            bullet_world: DiscreteDynamicsWorld::default(),
            world_collision_rb: Rc::default(),
        };

        // btDefaultCollisionConstructionInfo collisionConfigConstructionInfo = {};

        // _bulletWorldParams.collisionConfig.setup(collisionConfigConstructionInfo);

        // _bulletWorldParams.collisionDispatcher.setup(&_bulletWorldParams.collisionConfig);
        // _bulletWorldParams.constraintSolver = btSequentialImpulseConstraintSolver();

        // _bulletWorldParams.overlappingPairCache = new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache), 16)) btHashedOverlappingPairCache();
        // _bulletWorldParams.broadphase = btDbvtBroadphase(_bulletWorldParams.overlappingPairCache);

        // _bulletWorld.setup(
        //     &_bulletWorldParams.collisionDispatcher,
        //     &_bulletWorldParams.broadphase,
        //     &_bulletWorldParams.constraintSolver,
        //     &_bulletWorldParams.collisionConfig
        // );

        arena
            .bullet_world
            .set_gravity(arena.mutator_config.gravity * UU_TO_BT);

        // Adjust solver configuration to be closer to older Bullet (Rocket League's Bullet is from somewhere between 2013 and 2015)
        // auto& solverInfo = _bulletWorld.getSolverInfo();
        // solverInfo.m_splitImpulsePenetrationThreshold = 1.0e30f;
        // solverInfo.m_erp2 = 0.8f;

        // bool loadArenaStuff = gameMode != GameMode::THE_VOID;

        // if (loadArenaStuff) {
        if game_mode != GameMode::TheVoid {
            // _SetupArenaCollisionShapes();
            arena.setup_arena_collision_shapes();

            // Give arena collision shapes the proper restitution/friction values
            // for (size_t i = 0; i < _worldCollisionRBAmount; i++) {
            //     btRigidBody* rb = &_worldCollisionRBs[i];
            //     // TODO: Move to RLConst
            //     rb->setRestitution(0.3f);
            //     rb->setFriction(0.6f);
            //     rb->setRollingFriction(0.f);
            // }
        } else {
            // _worldCollisionRBs = NULL;
            // _worldCollisionRBAmount = 0;

            // _worldCollisionBvhShapes = NULL;
            // _worldCollisionPlaneShapes = NULL;
        }

        // { // Initialize ball
        //     ball = Ball::_AllocBall();

        //     ball->_BulletSetup(gameMode, &_bulletWorld, _mutatorConfig);
        //     ball->SetState(BallState());
        // }

        // if (loadArenaStuff) { // Initialize boost pads
        //     using namespace RLConst::BoostPads;

        //     bool isHoops = gameMode == GameMode::HOOPS;

        //     int amountSmall = isHoops ? LOCS_AMOUNT_SMALL_HOOPS : LOCS_AMOUNT_SMALL_SOCCAR;
        //     _boostPads.reserve(LOCS_AMOUNT_BIG + amountSmall);

        //     for (int i = 0; i < (LOCS_AMOUNT_BIG + amountSmall); i++) {
        //         bool isBig = i < LOCS_AMOUNT_BIG;

        //         btVector3 pos;
        //         if (isHoops) {
        //             pos = isBig ? LOCS_BIG_HOOPS[i] : LOCS_SMALL_HOOPS[i - LOCS_AMOUNT_BIG];
        //         } else {
        //             pos = isBig ? LOCS_BIG_SOCCAR[i] : LOCS_SMALL_SOCCAR[i - LOCS_AMOUNT_BIG];
        //         }

        //         BoostPad* pad = BoostPad::_AllocBoostPad();
        //         pad->_Setup(isBig, pos);

        //         _boostPads.push_back(pad);
        //         _boostPadGrid.Add(pad);
        //     }
        // }

        // // Set internal tick callback
        // _bulletWorld.setWorldUserInfo(this);

        // gContactAddedCallback = &Arena::_BulletContactAddedCallback;

        arena
    }

    pub fn step(&mut self) {
        // _bulletWorld.setWorldUserInfo(this);

        // { // Ball zero-vel sleeping
        //     if (ball->_rigidBody.m_linearVelocity.length2() == 0 && ball->_rigidBody.m_angularVelocity.length2() == 0) {
        //         ball->_rigidBody.setActivationState(ISLAND_SLEEPING);
        //     } else {
        //         ball->_rigidBody.setActivationState(ACTIVE_TAG);
        //     }
        // }

        // for (Car* car : _cars) {
        //     car->_PreTickUpdate(tickTime, _mutatorConfig, NULL);
        // }

        // if (gameMode == GameMode::SOCCAR) {
        //     for (BoostPad* pad : _boostPads)
        //         pad->_PreTickUpdate(tickTime);
        // }

        // // Update world
        // _bulletWorld.stepSimulation(tickTime, 0, tickTime);

        // for (Car* car : _cars) {
        //     car->_PostTickUpdate(tickTime, _mutatorConfig);
        //     car->_FinishPhysicsTick(_mutatorConfig);
        //     if (gameMode == GameMode::SOCCAR) {
        //         _boostPadGrid.CheckCollision(car);
        //     }
        // }

        // if (gameMode == GameMode::SOCCAR) {
        //     for (BoostPad* pad : _boostPads)
        //         pad->_PostTickUpdate(tickTime, _mutatorConfig);
        // }

        // ball->_FinishPhysicsTick(_mutatorConfig);

        // if (gameMode == GameMode::SOCCAR) {
        //     if (_goalScoreCallback.func != NULL) { // Potentially fire goal score callback
        //         float ballPosY = ball->_rigidBody.m_worldTransform.m_origin.y() * BT_TO_UU;
        //         if (abs(ballPosY) > RLConst::SOCCAR_BALL_SCORE_THRESHOLD_Y) {
        //             // Orange goal is at positive Y, so if the ball's Y is positive, it's in orange goal and thus blue scored
        //             Team scoringTeam = (ballPosY > 0) ? Team::BLUE : Team::ORANGE;
        //             _goalScoreCallback.func(this, scoringTeam, _goalScoreCallback.userInfo);
        //         }
        //     }
        // }

        // tickCount++;
    }

    #[inline]
    pub fn steps(&mut self, ticks_to_simulate: u64) {
        (0..ticks_to_simulate).for_each(|_| self.step());
    }
}
