use crate::bullet::collision::dispatch::collision_object::CollisionObject;
use glam::Affine3A;
use std::{cell::RefCell, rc::Rc};

pub struct CollisionObjectWrapper {
    pub object: Rc<RefCell<CollisionObject>>,
    pub world_transform: Affine3A,
}
