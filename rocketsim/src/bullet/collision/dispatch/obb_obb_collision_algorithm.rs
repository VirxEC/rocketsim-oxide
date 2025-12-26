use crate::bullet::collision::{
    broadphase::CollisionAlgorithm,
    dispatch::{
        box_box_detector::BoxBoxDetector, collision_object_wrapper::CollisionObjectWrapper,
    },
    narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
    shapes::collision_shape::CollisionShapes,
};

pub struct ObbObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_0_obj: CollisionObjectWrapper<'a>,
    compound_1_obj: CollisionObjectWrapper<'a>,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ObbObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_0_obj: CollisionObjectWrapper<'a>,
        compound_1_obj: CollisionObjectWrapper<'a>,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_0_obj,
            compound_1_obj,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for ObbObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(self) -> Option<PersistentManifold> {
        let CollisionShapes::Compound(compound_0_ref) =
            self.compound_0_obj.object.get_collision_shape()
        else {
            unreachable!();
        };

        let CollisionShapes::Compound(compound_1_ref) =
            self.compound_1_obj.object.get_collision_shape()
        else {
            unreachable!();
        };

        let org_0_trans = self.compound_0_obj.object.get_world_transform();
        let aabb_0 = compound_0_ref.get_aabb(org_0_trans);
        let org_1_trans = self.compound_1_obj.object.get_world_transform();
        let aabb_1 = compound_1_ref.get_aabb(org_1_trans);
        if !aabb_0.intersects(&aabb_1) {
            return None;
        }

        let child_0_trans = &compound_0_ref.child_transform;
        let child_0_world_trans = org_0_trans * child_0_trans;

        let child_1_trans = &compound_1_ref.child_transform;
        let child_1_world_trans = org_1_trans * child_1_trans;

        let mut detector = BoxBoxDetector {
            box1: &compound_0_ref.child_shape,
            col1: self.compound_0_obj.object,
            box2: &compound_1_ref.child_shape,
            col2: self.compound_1_obj.object,
            contact_added_callback: self.contact_added_callback,
        };

        detector.get_closest_points(child_0_world_trans, child_1_world_trans)
    }
}
