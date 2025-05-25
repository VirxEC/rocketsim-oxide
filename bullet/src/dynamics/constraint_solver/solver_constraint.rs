use glam::Vec3A;
use std::cell::RefCell;

pub enum SolverConstraintType {
    Contact1D = 0,
    Friction1D,
}

pub struct SolverConstraint {
    relpos1_cross_normal: Vec3A,
    contact_normal_1: Vec3A,
    relpos2_cross_normal: Vec3A,
    contact_normal_2: Vec3A,
    angular_component_a: Vec3A,
    angular_component_b: Vec3A,
    applied_push_impulse: RefCell<Vec3A>,
    applied_impulse: RefCell<Vec3A>,
    friction: f32,
    jac_diag_ab_inv: f32,
    rhs: f32,
    cfm: f32,
    lower_limit: f32,
    upper_limit: f32,
    rhs_penetration: f32,
    // union {
    //     void* m_originalContactPoint;
    //     btScalar m_unusedPadding4;
    //     int m_numRowsForNonContactConstraint;
    // };
    override_num_solver_iterations: i32,
    friction_index: i32,
    solver_body_id_a: i32,
    solver_body_id_b: i32,
    is_special: bool,
}
