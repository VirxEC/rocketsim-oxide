use super::{solver_body::SolverBody, solver_constraint::SolverConstraint};

pub type SingleConstraintRowSolver = fn(&mut SolverBody, &mut SolverBody, &SolverConstraint) -> f32;

pub struct SequentialImpulseConstraintSolver {
    pub tmp_solver_body_pool: Vec<SolverBody>,
    pub tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    pub tmp_solver_non_contact_constraint_pool: Vec<SolverConstraint>,
    pub tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    pub tmp_solver_contact_rolling_constraint_pool: Vec<SolverConstraint>,
    pub order_tmp_constraint_pool: Vec<i32>,
    pub order_non_contact_constraint_pool: Vec<i32>,
    pub order_friction_constraint_pool: Vec<i32>,
    pub max_override_num_solver_iterations: i32,
    pub fixed_body_id: i32,
    pub kinematic_body_unique_id_to_solver_body_table: Vec<i32>,
    pub resolve_single_constraint_row_generic: SingleConstraintRowSolver,
    pub resolve_single_constraint_row_lower_limit: SingleConstraintRowSolver,
    pub resolve_split_penetration_impulse: SingleConstraintRowSolver,
    pub cached_solver_mode: i32,
    pub least_squares_residual: f32,
    pub bt_seed_2: u64,
    // btSolverAnalyticsData m_analyticsData;
}

impl Default for SequentialImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_non_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            tmp_solver_contact_rolling_constraint_pool: Vec::new(),
            order_tmp_constraint_pool: Vec::new(),
            order_non_contact_constraint_pool: Vec::new(),
            order_friction_constraint_pool: Vec::new(),
            max_override_num_solver_iterations: 0,
            fixed_body_id: 0,
            kinematic_body_unique_id_to_solver_body_table: Vec::new(),
            resolve_single_constraint_row_generic: Self::resolve_single_constraint_row_generic,
            resolve_single_constraint_row_lower_limit:
                Self::resolve_single_constraint_row_lower_limit,
            resolve_split_penetration_impulse: Self::resolve_split_penetration_impulse,
            cached_solver_mode: 0,
            least_squares_residual: 0.0,
            bt_seed_2: 0,
        }
    }
}

impl SequentialImpulseConstraintSolver {
    pub fn resolve_single_constraint_row_generic(
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
        c: &SolverConstraint,
    ) -> f32 {
        todo!()
    }

    pub fn resolve_single_constraint_row_lower_limit(
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
        c: &SolverConstraint,
    ) -> f32 {
        todo!()
    }

    pub fn resolve_split_penetration_impulse(
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
        c: &SolverConstraint,
    ) -> f32 {
        todo!()
    }
}
