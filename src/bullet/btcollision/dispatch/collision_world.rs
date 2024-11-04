use std::{cell::RefCell, rc::Rc};

use crate::bullet::btdynamics::dynamics::rigid_body::RigidBody;

#[derive(Default)]
pub struct CollisionWorld {
    // broadphase_pair_cache: BroadphaseInterface,
    collision_objects: Vec<Rc<RefCell<RigidBody>>>,
}

impl CollisionWorld {
    // void btCollisionWorld::addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
    // {
    //     btAssert(collisionObject);

    //     //check that the object isn't already added
    //     btAssert(m_collisionObjects.findLinearSearch(collisionObject) == m_collisionObjects.size());
    //     btAssert(collisionObject->getWorldArrayIndex() == -1);  // do not add the same object to more than one collision world

    //     collisionObject->setWorldArrayIndex(m_collisionObjects.size());
    //     m_collisionObjects.push_back(collisionObject);

    //     //calculate new AABB
    //     btTransform trans = collisionObject->getWorldTransform();

    //     btVector3 minAabb;
    //     btVector3 maxAabb;
    //     collisionObject->getCollisionShape()->getAabb(trans, minAabb, maxAabb);

    //     int type = collisionObject->getCollisionShape()->getShapeType();
    //     collisionObject->setBroadphaseHandle(getBroadphase()->createProxy(
    //         minAabb,
    //         maxAabb,
    //         type,
    //         collisionObject,
    //         collisionFilterGroup,
    //         collisionFilterMask,
    //         m_dispatcher1));
    // }
    pub fn add_collision_object(
        &mut self,
        body: Rc<RefCell<RigidBody>>,
        collision_filter_group: i32,
        collision_filter_mask: i32,
    ) {
        let mut bol = body.borrow_mut();
        assert!(bol.collision_object.has_collision_shape());

        assert!(bol.collision_object.get_world_array_index().is_none());
        bol.collision_object
            .set_world_array_index(self.collision_objects.len());
        self.collision_objects.push(body.clone());

        // let trans = bol.collision_object.get_world_transform();
        // let aabb = bol.collision_object.get_collision_shape().get_aabb(trans);
        // let shape_type = bol.collision_object.get_collision_shape().get_shape_type();
    }
}
