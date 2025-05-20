use super::{
    optimized_bvh::OptimizedBvh, striding_mesh_interface::StridingMeshInterface,
    triangle_callback::TriangleCallback, triangle_info_map::TriangleInfoMap,
    triangle_mesh_shape::TriangleMeshShape,
};
use crate::collision::broadphase::{
    broadphase_proxy::BroadphaseNativeTypes, quantized_bvh::MyNodeOverlapCallback,
};
use glam::Vec3A;
use std::rc::Rc;

pub struct BvhTriangleMeshShape {
    triangle_mesh_shape: TriangleMeshShape,
    bvh: OptimizedBvh,
    triangle_info_map: Option<TriangleInfoMap>,
}

impl BvhTriangleMeshShape {
    pub fn new(
        mesh_interface: Rc<dyn StridingMeshInterface>,
        use_quantized_aabb_compression: bool,
    ) -> Self {
        let mut triangle_mesh_shape = TriangleMeshShape::new(mesh_interface.clone());
        triangle_mesh_shape.concave_shape.collision_shape.shape_type =
            BroadphaseNativeTypes::TriangleMeshShapeProxytype;

        Self {
            bvh: Self::build_optimized_bvh(
                &*mesh_interface,
                triangle_mesh_shape.local_aabb_min,
                triangle_mesh_shape.local_aabb_max,
                use_quantized_aabb_compression,
            ),
            triangle_mesh_shape,
            triangle_info_map: None,
        }
    }

    fn build_optimized_bvh(
        mesh_interface: &dyn StridingMeshInterface,
        local_aabb_min: Vec3A,
        local_aabb_max: Vec3A,
        use_quantized_aabb_compression: bool,
    ) -> OptimizedBvh {
        let mut bvh = OptimizedBvh::default();
        bvh.build(
            mesh_interface,
            local_aabb_min,
            local_aabb_max,
            use_quantized_aabb_compression,
        );

        bvh
    }

    pub const fn get_triangle_info_map(&self) -> Option<&TriangleInfoMap> {
        self.triangle_info_map.as_ref()
    }

    pub fn set_triangle_info_map(&mut self, triangle_info_map: TriangleInfoMap) {
        self.triangle_info_map = Some(triangle_info_map);
    }

    pub fn get_mesh_interface(&self) -> &dyn StridingMeshInterface {
        self.triangle_mesh_shape.mesh_interface.as_ref()
    }

    pub fn process_all_triangles(
        &self,
        callback: &mut dyn TriangleCallback,
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
}
