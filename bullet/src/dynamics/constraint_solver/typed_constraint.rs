pub struct TypedConstraint {
    object_type: i32,
    user_constraint_type: i32,
    // union {
    //     int m_userConstraintId;
    //     void* m_userConstraintPtr;
    // }
    breaking_impulse_threshold: f32,
    is_enabled: bool,
    needs_feedback: bool,
    override_num_solver_iterations: i32,
    // btRigidBody& m_rbA;
    // btRigidBody& m_rbB;
    applied_impulse: f32,
    // btJointFeedback* m_jointFeedback;
}
