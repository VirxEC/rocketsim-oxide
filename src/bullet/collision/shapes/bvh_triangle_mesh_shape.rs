use super::{
    collision_shape::CollisionShape, optimized_bvh::OptimizedBvh,
    triangle_callback::TriangleCallback, triangle_info_map::TriangleInfoMap,
    triangle_mesh::TriangleMesh, triangle_mesh_shape::TriangleMeshShape,
};
use crate::bullet::{
    collision::{
        broadphase::{broadphase_proxy::BroadphaseNativeTypes, bvh::MyNodeOverlapCallback},
        dispatch::internal_edge_utility::generate_internal_edge_info,
    },
    linear_math::aabb_util_2::Aabb,
};
use glam::{Affine3A, Vec3A};

pub struct BvhTriangleMeshShape {
    pub triangle_mesh_shape: TriangleMeshShape,
    bvh: OptimizedBvh,
    mesh_interface: TriangleMesh,
    triangle_info_map: TriangleInfoMap,
}

impl BvhTriangleMeshShape {
    pub fn new(mesh_interface: TriangleMesh) -> Self {
        let mut triangle_mesh_shape = TriangleMeshShape::new(&mesh_interface);
        triangle_mesh_shape.concave_shape.collision_shape.shape_type =
            BroadphaseNativeTypes::TriangleMeshShapeProxytype;

        // pre-calculate the aabb
        let trans = Affine3A::IDENTITY;
        let aabb = triangle_mesh_shape.get_aabb(&trans);
        triangle_mesh_shape
            .concave_shape
            .collision_shape
            .aabb_ident_cache = Some(aabb);
        triangle_mesh_shape.concave_shape.collision_shape.aabb_cache = Some(aabb);
        triangle_mesh_shape
            .concave_shape
            .collision_shape
            .aabb_cache_trans = trans;

        let bvh = OptimizedBvh::new(&mesh_interface, triangle_mesh_shape.local_aabb);
        let triangle_info_map = generate_internal_edge_info(&bvh.bvh, &mesh_interface);

        Self {
            triangle_mesh_shape,
            bvh,
            mesh_interface,
            triangle_info_map,
        }
    }

    #[must_use]
    pub const fn get_triangle_info_map(&self) -> &TriangleInfoMap {
        &self.triangle_info_map
    }

    #[must_use]
    pub const fn get_mesh_interface(&self) -> &TriangleMesh {
        &self.mesh_interface
    }

    pub fn process_all_triangles<T: TriangleCallback>(&self, callback: &mut T, aabb: &Aabb) {
        let mut my_node_callback = MyNodeOverlapCallback::new(self.get_mesh_interface(), callback);
        self.bvh
            .bvh
            .report_aabb_overlapping_node(&mut my_node_callback, aabb);
    }

    pub fn perform_raycast<T: TriangleCallback>(
        &self,
        callback: &mut T,
        ray_source: Vec3A,
        ray_target: Vec3A,
    ) {
        let mut my_node_callback = MyNodeOverlapCallback::new(self.get_mesh_interface(), callback);
        self.bvh
            .bvh
            .report_ray_overlapping_node(&mut my_node_callback, ray_source, ray_target);
    }

    #[must_use]
    pub const fn get_collision_shape(&self) -> &CollisionShape {
        &self.triangle_mesh_shape.concave_shape.collision_shape
    }
}
