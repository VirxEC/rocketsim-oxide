use super::solver_body::SolverBody;
use glam::Vec3A;

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
    pub fn resolve_single_constraint_row_generic(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = self.contact_normal_1.dot(body_a.delta_linear_velocity)
            + self
                .rel_pos1_cross_normal
                .dot(body_a.delta_angular_velocity);
        let delta_vel_2_dot_n = self.contact_normal_2.dot(body_b.delta_linear_velocity)
            + self
                .rel_pos2_cross_normal
                .dot(body_b.delta_angular_velocity);

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

        body_a.delta_linear_velocity +=
            self.contact_normal_1 * body_a.inv_mass * delta_impulse * body_a.linear_factor;
        body_a.delta_angular_velocity +=
            self.angular_component_a * delta_impulse * body_a.angular_factor;

        body_b.delta_linear_velocity +=
            self.contact_normal_2 * body_b.inv_mass * delta_impulse * body_b.linear_factor;
        body_b.delta_angular_velocity +=
            self.angular_component_b * delta_impulse * body_b.angular_factor;

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_single_constraint_row_lower_limit(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = self.contact_normal_1.dot(body_a.delta_linear_velocity)
            + self
                .rel_pos1_cross_normal
                .dot(body_a.delta_angular_velocity);
        let delta_vel_2_dot_n = self.contact_normal_2.dot(body_b.delta_linear_velocity)
            + self
                .rel_pos2_cross_normal
                .dot(body_b.delta_angular_velocity);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else {
            self.applied_impulse = sum;
        }

        body_a.delta_linear_velocity +=
            self.contact_normal_1 * body_a.inv_mass * delta_impulse * body_a.linear_factor;
        body_a.delta_angular_velocity +=
            self.angular_component_a * delta_impulse * body_a.angular_factor;

        body_b.delta_linear_velocity +=
            self.contact_normal_2 * body_b.inv_mass * delta_impulse * body_b.linear_factor;
        body_b.delta_angular_velocity +=
            self.angular_component_b * delta_impulse * body_b.angular_factor;

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

        let delta_vel_1_dot_n = self.contact_normal_1.dot(body_a.push_velocity)
            + self.rel_pos1_cross_normal.dot(body_a.turn_velocity);
        let delta_vel_2_dot_n = self.contact_normal_2.dot(body_b.push_velocity)
            + self.rel_pos2_cross_normal.dot(body_b.turn_velocity);

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_push_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_push_impulse;
            self.applied_push_impulse = self.lower_limit;
        } else {
            self.applied_push_impulse = sum;
        }

        body_a.push_velocity +=
            self.contact_normal_1 * body_a.inv_mass * delta_impulse * body_a.linear_factor;
        body_a.turn_velocity += self.angular_component_a * delta_impulse * body_a.angular_factor;

        body_b.push_velocity +=
            self.contact_normal_2 * body_b.inv_mass * delta_impulse * body_b.linear_factor;
        body_b.turn_velocity += self.angular_component_b * delta_impulse * body_b.angular_factor;

        delta_impulse / self.jac_diag_ab_inv
    }
}
