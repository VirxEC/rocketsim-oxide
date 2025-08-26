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
use std::{cell::RefCell, rc::Rc};

enum Algorithms<'a, T: ContactAddedCallback> {
    ConvexPlane(ConvexPlaneCollisionAlgorithm<'a, T>),
    ConvexConcave(ConvexConcaveCollisionAlgorithm<'a, T>),
    SphereObb(SphereObbCollisionAlgorithm<'a, T>),
    Compound(CompoundCollisionAlgorithm<'a, T>),
    ObbObb(ObbObbCollisionAlgorithm<'a, T>),
}

impl<'a, T: ContactAddedCallback> Algorithms<'a, T> {
    fn new_convex_plane(
        convex_obj: CollisionObjectWrapper,
        plane_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ConvexPlane(ConvexPlaneCollisionAlgorithm::new(
            convex_obj,
            plane_obj,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_convex_concave(
        convex_obj: Rc<RefCell<CollisionObject>>,
        concave_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ConvexConcave(ConvexConcaveCollisionAlgorithm::new(
            convex_obj,
            concave_obj,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_sphere_obb(
        sphere_obj: Rc<RefCell<CollisionObject>>,
        obb_obj: CollisionObjectWrapper,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::SphereObb(SphereObbCollisionAlgorithm::new(
            sphere_obj,
            obb_obj,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_obb_obb(
        compound_0_obj: CollisionObjectWrapper,
        compound_1_obj: CollisionObjectWrapper,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ObbObb(ObbObbCollisionAlgorithm::new(
            compound_0_obj,
            compound_1_obj,
            contact_added_callback,
        ))
    }

    const fn new_compound(
        compound_obj: Rc<RefCell<CollisionObject>>,
        other_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::Compound(CompoundCollisionAlgorithm::new(
            compound_obj,
            other_obj,
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
        col_obj_0: &CollisionObject,
        col_obj_0_ref: Rc<RefCell<CollisionObject>>,
        col_obj_1: &CollisionObject,
        col_obj_1_ref: Rc<RefCell<CollisionObject>>,
        contact_added_callback: &'a mut T,
    ) -> Algorithms<'a, T> {
        let shape0 = col_obj_0.get_collision_shape().unwrap().get_shape_type();
        let shape1 = col_obj_1.get_collision_shape().unwrap().get_shape_type();

        match shape0 {
            BroadphaseNativeTypes::StaticPlaneProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => {
                    let world_transform = *col_obj_1.get_world_transform();
                    Algorithms::new_convex_plane(
                        CollisionObjectWrapper {
                            object: col_obj_1_ref,
                            world_transform,
                        },
                        col_obj_0_ref,
                        true,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::CompoundShapeProxytype => Algorithms::new_compound(
                    col_obj_1_ref,
                    col_obj_0_ref,
                    true,
                    contact_added_callback,
                ),
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::SphereShapeProxytype => match shape1 {
                BroadphaseNativeTypes::StaticPlaneProxytype => {
                    let world_transform = *col_obj_0.get_world_transform();
                    Algorithms::new_convex_plane(
                        CollisionObjectWrapper {
                            object: col_obj_0_ref,
                            world_transform,
                        },
                        col_obj_1_ref,
                        false,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::TriangleMeshShapeProxytype => {
                    Algorithms::new_convex_concave(
                        col_obj_0_ref,
                        col_obj_1_ref,
                        false,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::CompoundShapeProxytype => {
                    let world_transform = *col_obj_1.get_world_transform();
                    Algorithms::new_sphere_obb(
                        col_obj_0_ref,
                        CollisionObjectWrapper {
                            object: col_obj_1_ref,
                            world_transform,
                        },
                        false,
                        contact_added_callback,
                    )
                }
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::TriangleMeshShapeProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => Algorithms::new_convex_concave(
                    col_obj_1_ref,
                    col_obj_0_ref,
                    true,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => Algorithms::new_compound(
                    col_obj_1_ref,
                    col_obj_0_ref,
                    true,
                    contact_added_callback,
                ),
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::CompoundShapeProxytype => match shape1 {
                BroadphaseNativeTypes::TriangleMeshShapeProxytype => Algorithms::new_compound(
                    col_obj_1_ref,
                    col_obj_0_ref,
                    false,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::SphereShapeProxytype => {
                    let world_transform = *col_obj_0.get_world_transform();
                    Algorithms::new_sphere_obb(
                        col_obj_1_ref,
                        CollisionObjectWrapper {
                            object: col_obj_0_ref,
                            world_transform,
                        },
                        true,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::StaticPlaneProxytype => Algorithms::new_compound(
                    col_obj_1_ref,
                    col_obj_0_ref,
                    false,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => {
                    let world_transform_0 = *col_obj_0.get_world_transform();
                    let world_transform_1 = *col_obj_1.get_world_transform();
                    Algorithms::new_obb_obb(
                        CollisionObjectWrapper {
                            object: col_obj_0_ref,
                            world_transform: world_transform_0,
                        },
                        CollisionObjectWrapper {
                            object: col_obj_1_ref,
                            world_transform: world_transform_1,
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
        proxy0: &RsBroadphaseProxy,
        proxy1: &RsBroadphaseProxy,
        contact_added_callback: &mut T,
    ) {
        let col_obj_0 = proxy0
            .broadphase_proxy
            .client_object
            .as_ref()
            .unwrap()
            .borrow();
        let col_obj_1 = proxy1
            .broadphase_proxy
            .client_object
            .as_ref()
            .unwrap()
            .borrow();

        if !col_obj_0.is_active() && !col_obj_1.is_active() {
            return;
        }

        let algorithm = Self::find_algorithm(
            &col_obj_0,
            proxy0
                .broadphase_proxy
                .client_object
                .as_ref()
                .unwrap()
                .clone(),
            &col_obj_1,
            proxy1
                .broadphase_proxy
                .client_object
                .as_ref()
                .unwrap()
                .clone(),
            contact_added_callback,
        );

        if let Some(manifold) = algorithm.process_collision(&col_obj_0, &col_obj_1) {
            self.manifolds.push(manifold);
        }
    }

    pub fn dispatch_all_collision_pairs<T: ContactAddedCallback>(
        &mut self,
        pair_cache: &mut RsBroadphase,
        contact_added_callback: &mut T,
    ) {
        pair_cache.process_all_overlapping_pairs(self, contact_added_callback);
    }
}
