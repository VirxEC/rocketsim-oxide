use std::{cell::RefCell, rc::Rc};

use super::{
    collision_object::CollisionObject,
    convex_concave_collision_algorithm::ConvexConcaveCollisionAlgorithm,
    convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
};
use crate::bullet::collision::{
    broadphase::{
        broadphase_proxy::BroadphaseNativeTypes,
        collision_algorithm::CollisionAlgorithm,
        rs_broadphase::{RsBroadphase, RsBroadphaseProxy},
    },
    dispatch::{
        collision_object_wrapper::CollisionObjectWrapper,
        compound_collision_algorithm::CompoundCollisionAlgorithm,
        obb_obb_collision_algorithm::ObbObbCollisionAlgorithm,
        sphere_obb_collision_algorithm::SphereObbCollisionAlgorithm,
    },
    narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
};

enum Algorithms<'a, T: ContactAddedCallback> {
    ConvexPlane(ConvexPlaneCollisionAlgorithm<'a, T>),
    ConvexConcave(ConvexConcaveCollisionAlgorithm<'a, T>),
    SphereObb(SphereObbCollisionAlgorithm<'a, T>),
    Compound(CompoundCollisionAlgorithm<'a, T>),
    ObbObb(ObbObbCollisionAlgorithm<'a, T>),
}

impl<'a, T: ContactAddedCallback> Algorithms<'a, T> {
    fn new_convex_plane(
        convex_obj: CollisionObjectWrapper<'a>,
        plane_obj: &'a CollisionObject,
        plane_obj_idx: usize,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ConvexPlane(ConvexPlaneCollisionAlgorithm::new(
            convex_obj,
            plane_obj,
            plane_obj_idx,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_convex_concave(
        convex_obj: &'a CollisionObject,
        convex_obj_idx: usize,
        concave_obj: &'a CollisionObject,
        plane_obj_idx: usize,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ConvexConcave(ConvexConcaveCollisionAlgorithm::new(
            convex_obj,
            convex_obj_idx,
            concave_obj,
            plane_obj_idx,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_sphere_obb(
        sphere_obj: &'a CollisionObject,
        sphere_obj_idx: usize,
        obb_obj: CollisionObjectWrapper<'a>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::SphereObb(SphereObbCollisionAlgorithm::new(
            sphere_obj,
            sphere_obj_idx,
            obb_obj,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_obb_obb(
        compound_0_obj: CollisionObjectWrapper<'a>,
        compound_1_obj: CollisionObjectWrapper<'a>,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ObbObb(ObbObbCollisionAlgorithm::new(
            compound_0_obj,
            compound_1_obj,
            contact_added_callback,
        ))
    }

    const fn new_compound(
        compound_obj: &'a CollisionObject,
        compound_obj_idx: usize,
        other_obj: &'a CollisionObject,
        other_obj_idx: usize,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::Compound(CompoundCollisionAlgorithm::new(
            compound_obj,
            compound_obj_idx,
            other_obj,
            other_obj_idx,
            is_swapped,
            contact_added_callback,
        ))
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for Algorithms<'_, T> {
    fn process_collision(
        self,
        body0: &CollisionObject,
        body1: &CollisionObject,
    ) -> Option<PersistentManifold> {
        match self {
            Self::ConvexPlane(alg) => alg.process_collision(body0, body1),
            Self::ConvexConcave(alg) => alg.process_collision(body0, body1),
            Self::SphereObb(alg) => alg.process_collision(body0, body1),
            Self::Compound(alg) => alg.process_collision(body0, body1),
            Self::ObbObb(alg) => alg.process_collision(body0, body1),
        }
    }
}

#[derive(Default)]
pub struct CollisionDispatcher {
    // dispatcher_flags: i32,
    pub manifolds: Vec<PersistentManifold>,
}

impl CollisionDispatcher {
    fn find_algorithm<'a, T: ContactAddedCallback>(
        col_obj_0: &'a CollisionObject,
        col_obj_0_idx: usize,
        col_obj_1: &'a CollisionObject,
        col_obj_1_idx: usize,
        contact_added_callback: &'a mut T,
    ) -> Algorithms<'a, T> {
        let shape0 = col_obj_0.get_collision_shape().unwrap().get_shape_type();
        let shape1 = col_obj_1.get_collision_shape().unwrap().get_shape_type();

        match shape0 {
            BroadphaseNativeTypes::StaticPlaneProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => {
                    Algorithms::new_convex_plane(
                        CollisionObjectWrapper {
                            index: col_obj_1_idx,
                            object: col_obj_1,
                            world_transform: *col_obj_1.get_world_transform(),
                        },
                        col_obj_0,
                        col_obj_0_idx,
                        true,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::CompoundShapeProxytype => Algorithms::new_compound(
                    col_obj_1,
                    col_obj_1_idx,
                    col_obj_0,
                    col_obj_0_idx,
                    true,
                    contact_added_callback,
                ),
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::SphereShapeProxytype => match shape1 {
                BroadphaseNativeTypes::StaticPlaneProxytype => {
                    Algorithms::new_convex_plane(
                        CollisionObjectWrapper {
                            index: col_obj_0_idx,
                            object: col_obj_0,
                            world_transform: *col_obj_0.get_world_transform(),
                        },
                        col_obj_1,
                        col_obj_1_idx,
                        false,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::TriangleMeshShapeProxytype => {
                    Algorithms::new_convex_concave(
                        col_obj_0,
                        col_obj_0_idx,
                        col_obj_1,
                        col_obj_1_idx,
                        false,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::CompoundShapeProxytype => {
                    Algorithms::new_sphere_obb(
                        col_obj_0,
                        col_obj_0_idx,
                        CollisionObjectWrapper {
                            index: col_obj_1_idx,
                            object: col_obj_1,
                            world_transform: *col_obj_1.get_world_transform(),
                        },
                        false,
                        contact_added_callback,
                    )
                }
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::TriangleMeshShapeProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => Algorithms::new_convex_concave(
                    col_obj_1,
                    col_obj_1_idx,
                    col_obj_0,
                    col_obj_0_idx,
                    true,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => Algorithms::new_compound(
                    col_obj_1,
                    col_obj_1_idx,
                    col_obj_0,
                    col_obj_0_idx,
                    true,
                    contact_added_callback,
                ),
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::CompoundShapeProxytype => match shape1 {
                BroadphaseNativeTypes::TriangleMeshShapeProxytype => Algorithms::new_compound(
                    col_obj_0,
                    col_obj_0_idx,
                    col_obj_1,
                    col_obj_1_idx,
                    false,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::SphereShapeProxytype => {
                    Algorithms::new_sphere_obb(
                        col_obj_1,
                        col_obj_1_idx,
                        CollisionObjectWrapper {
                            index: col_obj_0_idx,
                            object: col_obj_0,
                            world_transform: *col_obj_0.get_world_transform(),
                        },
                        true,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::StaticPlaneProxytype => Algorithms::new_compound(
                    col_obj_0,
                    col_obj_0_idx,
                    col_obj_1,
                    col_obj_1_idx,
                    false,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => {
                    Algorithms::new_obb_obb(
                        CollisionObjectWrapper {
                            index: col_obj_0_idx,
                            object: col_obj_0,
                            world_transform: *col_obj_0.get_world_transform(),
                        },
                        CollisionObjectWrapper {
                            index: col_obj_1_idx,
                            object: col_obj_1,
                            world_transform: *col_obj_1.get_world_transform(),
                        },
                        contact_added_callback,
                    )
                }
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
        }
    }

    pub fn near_callback<T: ContactAddedCallback>(
        &mut self,
        collision_objects: &[Rc<RefCell<CollisionObject>>],
        proxy0: &RsBroadphaseProxy,
        proxy1: &RsBroadphaseProxy,
        contact_added_callback: &mut T,
    ) {
        let col_obj_0_idx = proxy0.broadphase_proxy.client_object_idx.unwrap();
        let col_obj_1_idx = proxy1.broadphase_proxy.client_object_idx.unwrap();
        let col_obj_0 = collision_objects[col_obj_0_idx].borrow();
        let col_obj_1 = collision_objects[col_obj_1_idx].borrow();

        if !col_obj_0.is_active() && !col_obj_1.is_active()
            || !col_obj_0.has_contact_response()
            || !col_obj_1.has_contact_response()
        {
            return;
        }

        let algorithm = Self::find_algorithm(
            &col_obj_0,
            col_obj_0_idx,
            &col_obj_1,
            col_obj_1_idx,
            contact_added_callback,
        );

        if let Some(manifold) = algorithm.process_collision(&col_obj_0, &col_obj_1) {
            self.manifolds.push(manifold);
        }
    }

    pub fn dispatch_all_collision_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objects: &[Rc<RefCell<CollisionObject>>],
        pair_cache: &mut RsBroadphase,
        contact_added_callback: &mut T,
    ) {
        pair_cache.process_all_overlapping_pairs(collision_objects, self, contact_added_callback);
    }
}
