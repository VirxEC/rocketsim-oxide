use glam::Vec3A;

use crate::{
    BallState, GameMode, MutatorConfig,
    bullet::{
        collision::{
            broadphase::CollisionFilterGroups,
            dispatch::collision_object::CollisionFlags,
            shapes::{collision_shape::CollisionShapes, sphere_shape::SphereShape},
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
    },
    sim::{UserInfoTypes, collision_masks::CollisionMasks, consts},
};

pub(crate) struct Ball {
    pub state: BallState,
    pub rigid_body_idx: usize,
    pub ground_stick_applied: bool,
    pub velocity_impulse_cache: Vec3A,
}

impl Ball {
    fn make_ball_collision_shape(
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
    ) -> (CollisionShapes, Vec3A) {
        if game_mode == GameMode::Snowday {
            todo!()
        } else {
            let shape = SphereShape::new(mutator_config.ball_radius * consts::UU_TO_BT);
            let local_inertia = shape.calculate_local_inertia(mutator_config.ball_mass);

            (CollisionShapes::Sphere(shape), local_inertia)
        }
    }

    pub fn new(
        game_mode: GameMode,
        bullet_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        no_rot: bool,
    ) -> Self {
        let (collision_shape, local_inertia) =
            Self::make_ball_collision_shape(game_mode, mutator_config);

        let mut info =
            RigidBodyConstructionInfo::new(mutator_config.ball_mass, collision_shape, true);
        info.start_world_transform.translation.z = consts::ball::REST_Z * consts::UU_TO_BT;
        info.local_inertia = local_inertia;
        info.linear_damping = mutator_config.ball_drag;

        let coefs = if game_mode == GameMode::Snowday {
            consts::snowday::PUCK_COEFS
        } else {
            consts::ball::COEFS
        };
        info.friction = coefs.friction;
        info.restitution = coefs.restitution;

        let mut body = RigidBody::new(info);
        body.collision_object.user_index = UserInfoTypes::Ball;
        body.collision_object.collision_flags |= CollisionFlags::CustomMaterialCallback as u8;
        body.collision_object.no_rot = no_rot
            && matches!(
                body.collision_object.get_collision_shape(),
                CollisionShapes::Sphere(_)
            );

        let rigid_body_idx = bullet_world.add_rigid_body(
            body,
            CollisionFilterGroups::Default as u8
                | CollisionMasks::HoopsNet as u8
                | CollisionMasks::DropshotTile as u8,
            CollisionFilterGroups::All as u8,
        );

        Self {
            state: BallState::DEFAULT,
            rigid_body_idx,
            ground_stick_applied: false,
            velocity_impulse_cache: Vec3A::ZERO,
        }
    }
}
