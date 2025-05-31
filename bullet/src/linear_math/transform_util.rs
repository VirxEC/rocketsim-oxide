use glam::{Affine3A, Vec3A};

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

pub fn integrate_transform(
    cur_trans: &Affine3A,
    lin_vel: Vec3A,
    ang_vel: Vec3A,
    time_step: f32,
) -> Affine3A {
    let translation = cur_trans.translation + lin_vel * time_step;

    todo!();

    Affine3A {
        matrix3: cur_trans.matrix3,
        translation,
    }
}
