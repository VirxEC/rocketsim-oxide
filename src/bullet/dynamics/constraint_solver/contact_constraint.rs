use crate::bullet::dynamics::{
    constraint_solver::jacobian_entry::JacobianEntry, rigid_body::RigidBody,
};
use glam::Vec3A;

pub fn resolve_single_bilateral(
    body1: &RigidBody,
    pos1: Vec3A,
    body2: &RigidBody,
    pos2: Vec3A,
    normal: Vec3A,
) -> f32 {
    const CONTACT_DAMPING: f32 = -0.2;

    debug_assert!(normal.is_normalized());
    let body1_comt = *body1.collision_object.borrow().get_world_transform();
    let body2_comt = *body2.collision_object.borrow().get_world_transform();

    let rel_pos1 = pos1 - body1_comt.translation;
    let rel_pos2 = pos2 - body2_comt.translation;

    let vel1 = body1.get_velocity_in_local_point(rel_pos1);
    let vel2 = body2.get_velocity_in_local_point(rel_pos2);
    let vel = vel1 - vel2;

    let jac = JacobianEntry::new(
        &body1_comt.matrix3.transpose(),
        &body2_comt.matrix3.transpose(),
        rel_pos1,
        rel_pos2,
        normal,
        body1.inv_inertia_local,
        body1.inverse_mass,
        body2.inv_inertia_local,
        body2.inverse_mass,
    );

    let jac_diag_ab_inv = 1.0 / jac.get_diagonal();
    let rel_vel = normal.dot(vel);

    CONTACT_DAMPING * rel_vel * jac_diag_ab_inv
}
