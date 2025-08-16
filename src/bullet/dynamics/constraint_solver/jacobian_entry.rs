use glam::{Mat3A, Vec3A};

pub struct JacobianEntry {
    linear_joint_axis: Vec3A,
    a_j: Vec3A,
    b_j: Vec3A,
    min_v_jt_0: Vec3A,
    min_v_jt_1: Vec3A,
    a_diag: f32,
}

impl JacobianEntry {
    pub fn new(
        world2_a: &Mat3A,
        world2_b: &Mat3A,
        rel_pos1: Vec3A,
        rel_pos2: Vec3A,
        joint_axis: Vec3A,
        inertia_inv_a: Vec3A,
        mass_inv_a: f32,
        inertia_inv_b: Vec3A,
        mass_inv_b: f32,
    ) -> Self {
        let a_j = world2_a * rel_pos1.cross(joint_axis);
        let b_j = world2_b * rel_pos2.cross(-joint_axis);
        let min_v_jt_0 = inertia_inv_a * a_j;
        let min_v_jt_1 = inertia_inv_b * b_j;

        Self {
            a_j,
            b_j,
            min_v_jt_0,
            min_v_jt_1,
            linear_joint_axis: joint_axis,
            a_diag: mass_inv_a + min_v_jt_0.dot(a_j) + mass_inv_b + min_v_jt_1.dot(b_j),
        }
    }

    pub fn get_diagonal(&self) -> f32 {
        self.a_diag
    }
}
