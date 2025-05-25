use glam::{Affine3A, Vec3A};

pub struct SolverBody {
    world_transform: Affine3A,
    delta_linear_velocity: Vec3A,
    delta_angular_velocity: Vec3A,
    angular_factor: Vec3A,
    linear_factor: Vec3A,
    inv_mass: Vec3A,
    push_velocity: Vec3A,
    turn_velocity: Vec3A,
    linear_velocity: Vec3A,
    angular_velocity: Vec3A,
    external_force_impulse: Vec3A,
    external_torque_impulse: Vec3A,
    // btRigidBody* m_originalBody;
}
