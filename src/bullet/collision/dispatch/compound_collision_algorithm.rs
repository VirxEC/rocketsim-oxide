use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
            convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            box_shape::BoxShape, collision_shape::CollisionShapes,
            triangle_callback::TriangleCallback, triangle_shape::TriangleShape,
        },
    },
    linear_math::{
        AffineExt,
        aabb_util_2::{Aabb, test_aabb_against_aabb},
    },
};
use glam::Affine3A;
use std::{cell::RefCell, rc::Rc};

pub struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    convex_transform: Affine3A,
    pub aabb: &'a Aabb,
    box_shape: &'a BoxShape,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: CollisionObjectWrapper,
        tri_obj: Rc<RefCell<CollisionObject>>,
        aabb: &'a Aabb,
        box_shape: &'a BoxShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj.object, tri_obj, is_swapped),
            convex_transform: convex_obj.world_transform,
            aabb,
            is_swapped,
            box_shape,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> TriangleCallback for ConvexTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb: &Aabb,
        triangle_index: usize,
    ) -> bool {
        if !test_aabb_against_aabb(tri_aabb, self.aabb) {
            return true;
        }

        todo!();
        // convex-convex algorithm required

        // let (center, radius) = {
        //     let box_ref = self.manifold.body0.borrow();

        //     (
        //         box_ref.get_world_transform().translation,
        //         self.box_shape.get_radius(),
        //     )
        // };

        // let Some(contact_info) =
        //     triangle.intersect_sphere(center, radius, self.manifold.contact_breaking_threshold)
        // else {
        //     return true;
        // };

        // self.manifold.add_contact_point(
        //     contact_info.result_normal,
        //     contact_info.contact_point,
        //     contact_info.depth,
        //     -1,
        //     part_id as i32,
        //     -1,
        //     triangle_index as i32,
        // );

        true
    }
}

struct CompoundLeafCallback<'a, T: ContactAddedCallback> {
    compound_obj: Rc<RefCell<CollisionObject>>,
    other_obj: Rc<RefCell<CollisionObject>>,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> CompoundLeafCallback<'a, T> {
    pub const fn new(
        compound_obj: Rc<RefCell<CollisionObject>>,
        other_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_obj,
            other_obj,
            is_swapped,
            contact_added_callback,
        }
    }

    pub fn process_child_shape(&mut self) -> Option<PersistentManifold> {
        let compound_obj = self.compound_obj.borrow();
        let compound_col_shape = compound_obj.get_collision_shape().unwrap().borrow();
        let CollisionShapes::Compound(compound_shape) = &*compound_col_shape else {
            unreachable!()
        };

        let org_trans = *compound_obj.get_world_transform();

        let child = compound_shape.child.as_ref().unwrap();
        let child_trans = child.transform;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &child.child_shape;
        let aabb1 = box_shape.get_aabb(&new_child_world_trans);

        let other_obj = self.other_obj.borrow();
        let other_col_shape = other_obj.get_collision_shape().unwrap().borrow();
        let aabb2 = other_col_shape.get_aabb(other_obj.get_world_transform());

        if !test_aabb_against_aabb(&aabb1, &aabb2) {
            return None;
        }

        let compound_obj_wrap = CollisionObjectWrapper {
            object: self.compound_obj.clone(),
            world_transform: new_child_world_trans,
        };

        match &*other_col_shape {
            CollisionShapes::TriangleMesh(tri_mesh) => {
                let xform1 = other_obj.get_world_transform().transpose();
                let xform2 = new_child_world_trans;
                let convex_in_triangle_space = Affine3A {
                    matrix3: xform1.matrix3 * xform2.matrix3,
                    translation: xform1.transform_point3a(xform2.translation),
                };

                let aabb = box_shape.get_aabb(&convex_in_triangle_space);
                let mut convex_triangle_callback = ConvexTriangleCallback::new(
                    compound_obj_wrap,
                    self.other_obj.clone(),
                    &aabb,
                    box_shape,
                    self.is_swapped,
                    self.contact_added_callback,
                );

                tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);

                convex_triangle_callback.manifold.refresh_contact_points();
                if convex_triangle_callback.manifold.point_cache.is_empty() {
                    None
                } else {
                    Some(convex_triangle_callback.manifold)
                }
            }
            CollisionShapes::StaticPlane(_) => {
                let (body0, body1) = if self.is_swapped {
                    (self.other_obj.borrow(), self.compound_obj.borrow())
                } else {
                    (self.compound_obj.borrow(), self.other_obj.borrow())
                };

                ConvexPlaneCollisionAlgorithm::new(
                    compound_obj_wrap,
                    self.other_obj.clone(),
                    self.is_swapped,
                    self.contact_added_callback,
                )
                .process_collision(&body0, &body1)
            }
            _ => todo!(),
        }
    }
}

pub struct CompoundCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_obj: Rc<RefCell<CollisionObject>>,
    other_obj: Rc<RefCell<CollisionObject>>,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> CompoundCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_obj: Rc<RefCell<CollisionObject>>,
        other_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_obj,
            other_obj,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for CompoundCollisionAlgorithm<'_, T> {
    fn process_collision(
        self,
        _body0: &CollisionObject,
        _body1: &CollisionObject,
    ) -> Option<PersistentManifold> {
        let mut compound_leaf_callback = CompoundLeafCallback::new(
            self.compound_obj,
            self.other_obj,
            self.is_swapped,
            self.contact_added_callback,
        );

        compound_leaf_callback
            .process_child_shape()
            .and_then(|manifold| (!manifold.point_cache.is_empty()).then_some(manifold))
    }
}
