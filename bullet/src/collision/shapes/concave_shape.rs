use super::collision_shape::CollisionShape;

#[derive(Default)]
pub struct ConcaveShape {
    pub collision_shape: CollisionShape,
    pub collision_margin: f32,
}
