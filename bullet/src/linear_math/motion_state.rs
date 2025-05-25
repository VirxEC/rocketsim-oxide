use glam::Affine3A;

pub trait MotionState {
    fn get_world_transform(&self, world_trans: &mut Affine3A);
    fn set_world_tansform(&mut self, world_trans: Affine3A);
}
