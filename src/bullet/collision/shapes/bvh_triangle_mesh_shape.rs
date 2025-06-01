use super::{
    collision_shape::CollisionShape, optimized_bvh::OptimizedBvh,
    triangle_callback::TriangleCallback, triangle_info_map::TriangleInfoMap,
    triangle_mesh::TriangleMesh, triangle_mesh_shape::TriangleMeshShape,
};
use crate::bullet::collision::{
    broadphase::{broadphase_proxy::BroadphaseNativeTypes, quantized_bvh::MyNodeOverlapCallback},
    dispatch::internal_edge_utility::generate_internal_edge_info,
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
        let (aabb_min, aabb_max) = triangle_mesh_shape.get_aabb(&trans);
        triangle_mesh_shape
            .concave_shape
            .collision_shape
            .aabb_cached = true;
        triangle_mesh_shape
            .concave_shape
            .collision_shape
            .aabb_min_cache = aabb_min;
        triangle_mesh_shape
            .concave_shape
            .collision_shape
            .aabb_max_cache = aabb_max;
        triangle_mesh_shape
            .concave_shape
            .collision_shape
            .aabb_cache_trans = trans;

        let bvh = OptimizedBvh::new(
            &mesh_interface,
            triangle_mesh_shape.local_aabb_min,
            triangle_mesh_shape.local_aabb_max,
        );

        let triangle_info_map = generate_internal_edge_info(&bvh.quantized_bvh, &mesh_interface);

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

    pub fn process_all_triangles<T: TriangleCallback>(
        &self,
        callback: &mut T,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
    ) {
        let mut my_node_callback = MyNodeOverlapCallback::new(self.get_mesh_interface(), callback);
        self.bvh.quantized_bvh.report_aabb_overlapping_node(
            &mut my_node_callback,
            aabb_min,
            aabb_max,
        );
    }

    #[must_use]
    pub const fn get_collision_shape(&self) -> &CollisionShape {
        &self.triangle_mesh_shape.concave_shape.collision_shape
    }
}
