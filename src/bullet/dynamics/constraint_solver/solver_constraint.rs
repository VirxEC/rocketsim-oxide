use glam::Vec3A;

use super::solver_body::SolverBody;
use crate::bullet::{
    collision::narrowphase::manifold_point::ManifoldPoint,
    dynamics::{constraint_solver::contact_solver_info::ContactSolverInfo, rigid_body::RigidBody},
};

fn bullet_dot(vec0: Vec3A, vec1: Vec3A) -> f32 {
    let result = vec0 * vec1;
    result.x + (result.y + result.z)
}

#[derive(Default)]
pub struct SolverConstraint {
    pub rel_pos1_cross_normal: Vec3A,
    pub contact_normal_1: Vec3A,
    pub rel_pos2_cross_normal: Vec3A,
    pub contact_normal_2: Vec3A,
    pub angular_component_a: Vec3A,
    pub angular_component_b: Vec3A,
    pub applied_push_impulse: f32,
    pub applied_impulse: f32,
    pub friction: f32,
    pub jac_diag_ab_inv: f32,
    pub rhs: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub rhs_penetration: f32,
    pub friction_index: usize,
    pub solver_body_id_a: usize,
    pub solver_body_id_b: usize,
    pub is_special: bool,
}

impl SolverConstraint {
    pub fn restitution_curve(rel_vel: f32, restitution: f32, velocity_threshold: f32) -> f32 {
        if rel_vel.abs() < velocity_threshold {
            0.0
        } else {
            restitution * -rel_vel
        }
    }

    pub fn setup_contact_constraint(
        &mut self,
        info: &ContactSolverInfo,
        (solver_body_a, solver_body_b): (&mut SolverBody, &mut SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        cp: &ManifoldPoint,
    ) {
        let inv_time_step = 1.0 / info.time_step;
        let erp = info.erp_2;
        self.applied_impulse = cp.applied_impulse * info.warmstarting_factor;

        let (denom0, vel0, vel_1_dot_n) = rb0.map_or((0.0, Vec3A::ZERO, 0.0), |rb| {
            let torque_axis = rel_pos1.cross(cp.normal_world_on_b);
            let angular_component = rb.inertia_tensor_world * torque_axis;
            let vec = angular_component.cross(rel_pos1);
            let denom = rb.inverse_mass + cp.normal_world_on_b.dot(vec);
            let vel = rb.get_velocity_in_local_point(rel_pos1);

            let (contact_normal, rel_pos_cross_normal) = (cp.normal_world_on_b, torque_axis);

            solver_body_a.internal_apply_impulse(
                contact_normal * solver_body_a.inv_mass,
                angular_component,
                self.applied_impulse,
            );

            let vel_dot_n = contact_normal
                .dot(solver_body_a.linear_velocity + solver_body_a.external_force_impulse)
                + rel_pos_cross_normal
                    .dot(solver_body_a.angular_velocity + solver_body_a.external_torque_impulse);

            self.angular_component_a = angular_component;
            self.contact_normal_1 = contact_normal;
            self.rel_pos1_cross_normal = rel_pos_cross_normal;

            (denom, vel, vel_dot_n)
        });

        let (denom1, vel1, vel_2_dot_n) = rb1.map_or((0.0, Vec3A::ZERO, 0.0), |rb| {
            let torque_axis = rel_pos2.cross(-cp.normal_world_on_b);
            let angular_component = rb.inertia_tensor_world * torque_axis;
            let vec = angular_component.cross(rel_pos2);
            let denom = rb.inverse_mass + (-cp.normal_world_on_b).dot(vec);
            let vel = rb.get_velocity_in_local_point(rel_pos2);
            let (contact_normal, rel_pos_cross_normal) = (-cp.normal_world_on_b, -torque_axis);

            solver_body_b.internal_apply_impulse(
                -contact_normal * solver_body_b.inv_mass,
                -angular_component,
                -self.applied_impulse,
            );

            let vel_dot_n = contact_normal
                .dot(solver_body_b.linear_velocity + solver_body_b.external_force_impulse)
                + rel_pos_cross_normal
                    .dot(solver_body_b.angular_velocity + solver_body_b.external_torque_impulse);

            self.angular_component_b = angular_component;
            self.contact_normal_2 = contact_normal;
            self.rel_pos2_cross_normal = rel_pos_cross_normal;

            (denom, vel, vel_dot_n)
        });

        self.jac_diag_ab_inv = info.sor / (denom0 + denom1);

        let vel = vel0 - vel1;
        let rel_vel = cp.normal_world_on_b.dot(vel);
        let restitution = Self::restitution_curve(
            rel_vel,
            cp.combined_restitution,
            info.restitution_velocity_threshold,
        )
        .max(0.0);

        let penetration = cp.distance_1 + info.linear_slop;
        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let rel_vel = vel_1_dot_n + vel_2_dot_n;
        let velocity_error = restitution - rel_vel;

        let penetration_impulse = positional_error * self.jac_diag_ab_inv;
        let velocity_impulse = velocity_error * self.jac_diag_ab_inv;

        (self.rhs, self.rhs_penetration) = if penetration > info.split_impulse_penetration_threshold
        {
            (penetration_impulse + velocity_impulse, 0.0)
        } else {
            (velocity_impulse, penetration_impulse)
        };
    }

    pub fn setup_friction_constraint(
        &mut self,
        (solver_body_a, solver_body_b): (&SolverBody, &SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        normal_axis: Vec3A,
        relaxation: f32,
    ) {
        let (vel_1_dot_n, denom1) = rb0.map_or((0.0, 0.0), |rb| {
            let torque_axis = rel_pos1.cross(normal_axis);

            let vel_dot_n = normal_axis
                .dot(solver_body_a.linear_velocity + solver_body_a.external_force_impulse)
                + torque_axis.dot(solver_body_a.angular_velocity);

            let angular_component = rb.inertia_tensor_world * torque_axis;
            let vec = angular_component.cross(rel_pos1);
            let denom = rb.inverse_mass + normal_axis.dot(vec);

            self.contact_normal_1 = normal_axis;
            self.rel_pos1_cross_normal = torque_axis;
            self.angular_component_a = angular_component;

            (vel_dot_n, denom)
        });

        let (vel_2_dot_n, denom2) = rb1.map_or((0.0, 0.0), |rb| {
            let normal_axis = -normal_axis;
            let torque_axis = rel_pos2.cross(normal_axis);

            let vel_dot_n = normal_axis
                .dot(solver_body_b.linear_velocity + solver_body_b.external_force_impulse)
                + torque_axis.dot(solver_body_b.angular_velocity);

            let angular_component = rb.inertia_tensor_world * torque_axis;
            let vec = angular_component.cross(rel_pos2);
            let denom = rb.inverse_mass + normal_axis.dot(vec);

            self.contact_normal_2 = normal_axis;
            self.rel_pos2_cross_normal = torque_axis;
            self.angular_component_b = angular_component;

            (vel_dot_n, denom)
        });

        self.jac_diag_ab_inv = relaxation / (denom1 + denom2);

        let rel_vel = vel_1_dot_n + vel_2_dot_n;
        let velocity_error = -rel_vel;
        self.rhs = velocity_error * self.jac_diag_ab_inv;
    }

    pub fn resolve_single_constraint_row_generic(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = bullet_dot(self.contact_normal_1, body_a.delta_linear_velocity)
            + bullet_dot(self.rel_pos1_cross_normal, body_a.delta_angular_velocity);
        let delta_vel_2_dot_n = bullet_dot(self.contact_normal_2, body_b.delta_linear_velocity)
            + bullet_dot(self.rel_pos2_cross_normal, body_b.delta_angular_velocity);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else if sum > self.upper_limit {
            delta_impulse = self.upper_limit - self.applied_impulse;
            self.applied_impulse = self.upper_limit;
        } else {
            self.applied_impulse = sum;
        }

        body_a.delta_linear_velocity += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
        body_a.delta_angular_velocity += self.angular_component_a * delta_impulse;

        body_b.delta_linear_velocity += self.contact_normal_2 * body_b.inv_mass * delta_impulse;
        body_b.delta_angular_velocity += self.angular_component_b * delta_impulse;

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_single_constraint_row_lower_limit(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = bullet_dot(self.contact_normal_1, body_a.delta_linear_velocity)
            + bullet_dot(self.rel_pos1_cross_normal, body_a.delta_angular_velocity);
        let delta_vel_2_dot_n = bullet_dot(self.contact_normal_2, body_b.delta_linear_velocity)
            + bullet_dot(self.rel_pos2_cross_normal, body_b.delta_angular_velocity);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else {
            self.applied_impulse = sum;
        }

        body_a.delta_linear_velocity += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
        body_a.delta_angular_velocity += self.angular_component_a * delta_impulse;

        body_b.delta_linear_velocity += self.contact_normal_2 * body_b.inv_mass * delta_impulse;
        body_b.delta_angular_velocity += self.angular_component_b * delta_impulse;

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_split_penetration_impulse(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        if self.rhs_penetration == 0.0 {
            return 0.0;
        }

        let mut delta_impulse = self.rhs_penetration;

        let delta_vel_1_dot_n = bullet_dot(self.contact_normal_1, body_a.push_velocity)
            + bullet_dot(self.rel_pos1_cross_normal, body_a.turn_velocity);
        let delta_vel_2_dot_n = bullet_dot(self.contact_normal_2, body_b.push_velocity)
            + bullet_dot(self.rel_pos2_cross_normal, body_b.turn_velocity);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_push_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_push_impulse;
            self.applied_push_impulse = self.lower_limit;
        } else {
            self.applied_push_impulse = sum;
        }

        body_a.push_velocity += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
        body_a.turn_velocity += self.angular_component_a * delta_impulse;

        body_b.push_velocity += self.contact_normal_2 * body_b.inv_mass * delta_impulse;
        body_b.turn_velocity += self.angular_component_b * delta_impulse;

        delta_impulse / self.jac_diag_ab_inv
    }
}
