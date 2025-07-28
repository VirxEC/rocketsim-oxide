use glam::{Affine3A, Mat3A, Quat, Vec3A, Vec4, Vec4Swizzles};
use std::f32::consts::FRAC_PI_4;

const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

pub fn integrate_transform_no_rot(
    cur_trans: &Affine3A,
    lin_vel: Vec3A,
    time_step: f32,
) -> Affine3A {
    Affine3A {
        matrix3: cur_trans.matrix3,
        translation: cur_trans.translation + lin_vel * time_step,
    }
}

/// A reimplementation of bullet's Quat * Quat algorithm,
/// since using glam's results in some minute differences
fn bullet_quat_mul(q1: Quat, q2: Quat) -> Quat {
    const NEG_W: Vec4 = Vec4::new(1.0, 1.0, 1.0, -1.0);

    let q1: Vec4 = q1.into();
    let q2: Vec4 = q2.into();

    let a2 = q1.yzxy() * q2.zxyy();
    let a1 = q1.xyzx() * q2.wwwx() + a2;

    let b1 = q1.zxyz() * q2.yzxz();
    let a0 = q1.wwww() * q2 - b1;

    let q = a0 + a1 * NEG_W;
    Quat::from_array(q.to_array())
}

pub fn integrate_transform(
    cur_trans: &Affine3A,
    lin_vel: Vec3A,
    ang_vel: Vec3A,
    time_step: f32,
) -> Affine3A {
    let translation = cur_trans.translation + lin_vel * time_step;

    let mut angle = ang_vel.length();

    if angle * time_step > ANGULAR_MOTION_THRESHOLD {
        angle = ANGULAR_MOTION_THRESHOLD / time_step;
    }

    let axis = if angle < 0.001 {
        ang_vel
            * (0.5 * time_step - time_step * time_step * time_step * 0.020833334)
            * angle
            * angle
    } else {
        ang_vel * ((0.5 * angle * time_step).sin() / angle)
    };

    let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, (angle * time_step * 0.5).cos());
    let orn0 = Quat::from_mat3a(&cur_trans.matrix3);
    let predicted_orn = bullet_quat_mul(dorn, orn0).normalize();
    let matrix3 = Mat3A::from_quat(predicted_orn);

    Affine3A {
        matrix3,
        translation,
    }
}
