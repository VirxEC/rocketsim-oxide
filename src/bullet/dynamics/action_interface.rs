use crate::bullet::collision::dispatch::collision_world::CollisionWorld;

pub trait ActionInterface {
    fn update_action(&mut self, collision_world: &mut CollisionWorld, delta_time_step: f32);
}
