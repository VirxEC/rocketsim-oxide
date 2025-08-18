use crate::bullet::{
    collision::dispatch::collision_object::CollisionObject,
    dynamics::{
        constraint_solver::{
            contact_solver_info::ContactSolverInfo,
            jacobian_entry::{JacbobianBody, get_jacobian_diagonal},
        },
        rigid_body::RigidBody,
    },
};
use glam::Vec3A;

pub struct ContactBody<'a> {
    pub rb: &'a RigidBody,
    pub co: &'a CollisionObject,
}

pub fn resolve_single_collision(
    body1: &ContactBody,
    body2: &ContactBody,
    contact_position_world: Vec3A,
    contact_normal_on_b: Vec3A,
    solver_info: &ContactSolverInfo,
    distance: f32,
) -> f32 {
    let rel_pos1 = contact_position_world - body1.co.get_world_transform().translation;
    let rel_pos2 = contact_position_world - body2.co.get_world_transform().translation;

    let vel1 = body1.rb.get_velocity_in_local_point(rel_pos1);
    let vel2 = body2.rb.get_velocity_in_local_point(rel_pos2);
    let vel = vel1 - vel2;
    let rel_vel = contact_normal_on_b.dot(vel);

    let positional_error = solver_info.erp * -distance / solver_info.time_step;
    let velocity_error = -rel_vel;
    let denom0 = body1
        .rb
        .compute_impulse_denominator(contact_position_world, contact_normal_on_b);
    let denom1 = body2
        .rb
        .compute_impulse_denominator(contact_position_world, contact_normal_on_b);
    let jac_diag_ab_inv = 1.0 / (denom0 + denom1);

    let penetration_impulse = positional_error * jac_diag_ab_inv;
    let velocity_impulse = velocity_error * jac_diag_ab_inv;

    let normal_impulse = penetration_impulse + velocity_impulse;
    normal_impulse.max(0.0)
}

pub fn resolve_single_bilateral(
    body1: &ContactBody,
    body2: &ContactBody,
    pos1: Vec3A,
    pos2: Vec3A,
    normal: Vec3A,
) -> f32 {
    const CONTACT_DAMPING: f32 = -0.2;

    debug_assert!(normal.is_normalized());
    let body1_comt = body1.co.get_world_transform();
    let body2_comt = body2.co.get_world_transform();

    let rel_pos1 = pos1 - body1_comt.translation;
    let rel_pos2 = pos2 - body2_comt.translation;

    let vel1 = body1.rb.get_velocity_in_local_point(rel_pos1);
    let vel2 = body2.rb.get_velocity_in_local_point(rel_pos2);
    let vel = vel1 - vel2;

    let jac_body_a = JacbobianBody {
        world: &body1_comt.matrix3.transpose(),
        rel_pos: rel_pos1,
        inertia_inv: body1.rb.inv_inertia_local,
        mass_inv: body1.rb.inverse_mass,
    };

    let jac_body_b = JacbobianBody {
        world: &body2_comt.matrix3.transpose(),
        rel_pos: rel_pos2,
        inertia_inv: body2.rb.inv_inertia_local,
        mass_inv: body2.rb.inverse_mass,
    };

    let jac_diag_ab = get_jacobian_diagonal(jac_body_a, jac_body_b, normal);
    let jac_diag_ab_inv = 1.0 / jac_diag_ab;
    let rel_vel = normal.dot(vel);

    CONTACT_DAMPING * rel_vel * jac_diag_ab_inv
}
