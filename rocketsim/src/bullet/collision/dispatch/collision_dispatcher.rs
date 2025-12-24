use super::{
    convex_concave_collision_algorithm::ConvexConcaveCollisionAlgorithm,
    convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
};
use crate::bullet::{
    collision::{
        broadphase::{
            BroadphaseNativeTypes, CollisionAlgorithm, GridBroadphase, GridBroadphaseProxy,
        },
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
            compound_collision_algorithm::CompoundCollisionAlgorithm,
            obb_obb_collision_algorithm::ObbObbCollisionAlgorithm,
            sphere_obb_collision_algorithm::SphereObbCollisionAlgorithm,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
    },
    dynamics::rigid_body::RigidBody,
};

enum Algorithms<'a, T: ContactAddedCallback> {
    ConvexPlane(ConvexPlaneCollisionAlgorithm<'a, T>),
    ConvexConcave(ConvexConcaveCollisionAlgorithm<'a, T>),
    SphereObb(SphereObbCollisionAlgorithm<'a, T>),
    Compound(CompoundCollisionAlgorithm<'a, T>),
    ObbObb(ObbObbCollisionAlgorithm<'a, T>),
}

impl<'a, T: ContactAddedCallback> Algorithms<'a, T> {
    const fn new_convex_plane(
        convex_obj: CollisionObjectWrapper<'a>,
        plane_obj: &'a CollisionObject,
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
        convex_obj: &'a CollisionObject,
        concave_obj: &'a CollisionObject,
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
        sphere_obj: &'a CollisionObject,
        obb_obj: CollisionObjectWrapper<'a>,
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
        other_obj: &'a CollisionObject,
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
    fn process_collision(self) -> Option<PersistentManifold> {
        match self {
            Self::ConvexPlane(alg) => alg.process_collision(),
            Self::ConvexConcave(alg) => alg.process_collision(),
            Self::SphereObb(alg) => alg.process_collision(),
            Self::Compound(alg) => alg.process_collision(),
            Self::ObbObb(alg) => alg.process_collision(),
        }
    }
}

#[derive(Default)]
pub struct CollisionDispatcher {
    pub manifolds: Vec<PersistentManifold>,
}

impl CollisionDispatcher {
    fn find_algorithm<'a, T: ContactAddedCallback>(
        col_obj_0: &'a CollisionObject,
        col_obj_1: &'a CollisionObject,
        contact_added_callback: &'a mut T,
    ) -> Algorithms<'a, T> {
        let shape0 = col_obj_0.get_collision_shape().get_shape_type();
        let shape1 = col_obj_1.get_collision_shape().get_shape_type();

        match shape0 {
            BroadphaseNativeTypes::StaticPlaneProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => Algorithms::new_convex_plane(
                    CollisionObjectWrapper {
                        object: col_obj_1,
                        world_transform: *col_obj_1.get_world_transform(),
                    },
                    col_obj_0,
                    true,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => {
                    Algorithms::new_compound(col_obj_1, col_obj_0, true, contact_added_callback)
                }
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::SphereShapeProxytype => match shape1 {
                BroadphaseNativeTypes::StaticPlaneProxytype => Algorithms::new_convex_plane(
                    CollisionObjectWrapper {
                        object: col_obj_0,
                        world_transform: *col_obj_0.get_world_transform(),
                    },
                    col_obj_1,
                    false,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::TriangleMeshShapeProxytype => {
                    Algorithms::new_convex_concave(
                        col_obj_0,
                        col_obj_1,
                        false,
                        contact_added_callback,
                    )
                }
                BroadphaseNativeTypes::CompoundShapeProxytype => Algorithms::new_sphere_obb(
                    col_obj_0,
                    CollisionObjectWrapper {
                        object: col_obj_1,
                        world_transform: *col_obj_1.get_world_transform(),
                    },
                    false,
                    contact_added_callback,
                ),
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::TriangleMeshShapeProxytype => match shape1 {
                BroadphaseNativeTypes::SphereShapeProxytype => Algorithms::new_convex_concave(
                    col_obj_1,
                    col_obj_0,
                    true,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => {
                    Algorithms::new_compound(col_obj_1, col_obj_0, true, contact_added_callback)
                }
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            BroadphaseNativeTypes::CompoundShapeProxytype => match shape1 {
                BroadphaseNativeTypes::StaticPlaneProxytype
                | BroadphaseNativeTypes::TriangleMeshShapeProxytype => {
                    Algorithms::new_compound(col_obj_0, col_obj_1, false, contact_added_callback)
                }
                BroadphaseNativeTypes::SphereShapeProxytype => Algorithms::new_sphere_obb(
                    col_obj_1,
                    CollisionObjectWrapper {
                        object: col_obj_0,
                        world_transform: *col_obj_0.get_world_transform(),
                    },
                    true,
                    contact_added_callback,
                ),
                BroadphaseNativeTypes::CompoundShapeProxytype => Algorithms::new_obb_obb(
                    CollisionObjectWrapper {
                        object: col_obj_0,
                        world_transform: *col_obj_0.get_world_transform(),
                    },
                    CollisionObjectWrapper {
                        object: col_obj_1,
                        world_transform: *col_obj_1.get_world_transform(),
                    },
                    contact_added_callback,
                ),
                _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
            },
            _ => todo!("shape0/shape1: {shape0:?}/{shape1:?}"),
        }
    }

    pub fn near_callback<T: ContactAddedCallback>(
        &mut self,
        collision_objects: &[RigidBody],
        proxy0: &GridBroadphaseProxy,
        proxy1: &GridBroadphaseProxy,
        contact_added_callback: &mut T,
    ) {
        let rb0 = &collision_objects[proxy0.client_object_idx];
        let rb1 = &collision_objects[proxy1.client_object_idx];

        if !rb0.collision_object.is_active() && !rb1.collision_object.is_active()
            || !rb0.collision_object.has_contact_response()
            || !rb1.collision_object.has_contact_response()
        {
            return;
        }

        let algorithm = Self::find_algorithm(
            &rb0.collision_object,
            &rb1.collision_object,
            contact_added_callback,
        );

        if let Some(manifold) = algorithm.process_collision() {
            self.manifolds.push(manifold);
        }
    }

    pub fn dispatch_all_collision_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objects: &[RigidBody],
        pair_cache: &mut GridBroadphase,
        contact_added_callback: &mut T,
    ) {
        pair_cache.process_all_overlapping_pairs(collision_objects, self, contact_added_callback);
    }
}
