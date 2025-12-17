use std::{cell::RefCell, rc::Rc};

use glam::Affine3A;

use crate::bullet::collision::dispatch::collision_object::CollisionObject;

pub struct CollisionObjectWrapper<'a> {
    pub index: usize,
    pub object: &'a CollisionObject,
    pub world_transform: Affine3A,
}
