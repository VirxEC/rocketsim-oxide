use super::collision_object::CollisionObject;
use crate::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        narrowphase::persistent_manifold::PersistentManifold,
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::AffineTranspose,
};
use std::{cell::RefCell, rc::Rc};

pub struct ConvexPlaneCollisionAlgorithm {
    manifold: PersistentManifold,
    num_perturbation_iterations: i32,
    minimum_pointers_perturbation_threshold: i32,
    is_swapped: bool,
}

impl ConvexPlaneCollisionAlgorithm {
    pub fn new(
        convex_obj: Rc<RefCell<CollisionObject>>,
        plane_obj: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
    ) -> Self {
        Self {
            is_swapped,
            manifold: PersistentManifold::new(convex_obj, plane_obj, is_swapped),
            num_perturbation_iterations: 1,
            minimum_pointers_perturbation_threshold: 0,
        }
    }
}

impl CollisionAlgorithm for ConvexPlaneCollisionAlgorithm {
    fn into_manifold(self) -> PersistentManifold {
        self.manifold
    }

    fn process_collision(&mut self, body0: &CollisionObject, body1: &CollisionObject) {
        let (convex_obj, plane_obj) = if self.is_swapped {
            (body1, body0)
        } else {
            (body0, body1)
        };

        let sphere_col_shape = convex_obj.get_collision_shape().unwrap().borrow();
        let CollisionShapes::Sphere(sphere_shape) = &*sphere_col_shape else {
            unreachable!()
        };
        // let convex_shape = &sphere_shape.convex_internal_shape.convex_shape;

        let plane_col_shape = plane_obj.get_collision_shape().unwrap().borrow();
        let CollisionShapes::StaticPlane(plane_shape) = &*plane_col_shape else {
            unreachable!()
        };

        let plane_normal = plane_shape.get_plane_normal();
        let plane_constant = plane_shape.get_plane_constant();

        let plane_in_convex =
            convex_obj.get_world_transform().transpose() * *plane_obj.get_world_transform();
        let convex_in_plane_trans =
            plane_obj.get_world_transform().transpose() * *convex_obj.get_world_transform();

        let vtx = sphere_shape.local_get_support_vertex(plane_in_convex.matrix3 * -plane_normal);
        let vtx_in_plane = convex_in_plane_trans.transform_point3a(vtx);
        let distance = plane_normal.dot(vtx_in_plane) - plane_constant;

        if distance < self.manifold.contact_breaking_threshold {
            let vtx_in_plane_projected = vtx_in_plane - distance * plane_normal;
            let vtx_in_plane_in_world = plane_obj
                .get_world_transform()
                .transform_point3a(vtx_in_plane_projected);
            let normal_on_surface_b = plane_obj.get_world_transform().matrix3 * plane_normal;

            self.manifold.add_contact_point(
                normal_on_surface_b,
                vtx_in_plane_in_world,
                distance,
                -1,
                -1,
                -1,
                -1,
            );
        }

        // the perturbation algorithm doesn't work well with implicit surfaces such as spheres, cylinder and cones:
        // they keep on rolling forever because of the additional off-center contact points
        // so only enable the feature for polyhedral shapes (btBoxShape, btConvexHullShape etc)
        // if (convexShape->isPolyhedral() && resultOut->getPersistentManifold()->getNumContacts() < m_minimumPointsPerturbationThreshold)
        // {
        // 	btVector3 v0, v1;
        // 	btPlaneSpace1(planeNormal, v0, v1);
        // 	//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

        // 	const btScalar angleLimit = 0.125f * SIMD_PI;
        // 	btScalar perturbeAngle;
        // 	btScalar radius = convexShape->getAngularMotionDisc();
        // 	perturbeAngle = gContactBreakingThreshold / radius;
        // 	if (perturbeAngle > angleLimit)
        // 		perturbeAngle = angleLimit;

        // 	btQuaternion perturbeRot(v0, perturbeAngle);
        // 	for (int i = 0; i < m_numPerturbationIterations; i++)
        // 	{
        // 		btScalar iterationAngle = i * (SIMD_2_PI / btScalar(m_numPerturbationIterations));
        // 		btQuaternion rotq(planeNormal, iterationAngle);
        // 		collideSingleContact(rotq.inverse() * perturbeRot * rotq, body0Wrap, body1Wrap, dispatchInfo, resultOut);
        // 	}
        // }

        self.manifold.refresh_contact_points();
    }
}
