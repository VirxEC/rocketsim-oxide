use super::{
    concave_shape::ConcaveShape, striding_mesh_interface::StridingMeshInterface,
    triangle_callback::TriangleCallback,
};
use crate::{
    collision::shapes::triangle_callback::InternalTriangleIndexCallback,
    linear_math::aabb_util_2::test_aabb_against_aabb,
};
use glam::{Affine3A, Vec3A};
use std::sync::Arc;

struct FilteredCallback<'a> {
    callback: &'a mut dyn TriangleCallback,
    aabb_min: Vec3A,
    aabb_max: Vec3A,
}

impl InternalTriangleIndexCallback for FilteredCallback<'_> {
    fn internal_process_triangle_index(
        &mut self,
        triangle: &[Vec3A],
        tri_aabb_min: Vec3A,
        tri_aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool {
        if test_aabb_against_aabb(tri_aabb_min, tri_aabb_max, self.aabb_min, self.aabb_max) {
            self.callback
                .process_triangle(triangle, part_id, triangle_index)
        } else {
            true
        }
    }
}

impl<'a> FilteredCallback<'a> {
    fn new(callback: &'a mut dyn TriangleCallback, aabb_min: Vec3A, aabb_max: Vec3A) -> Self {
        Self {
            callback,
            aabb_min,
            aabb_max,
        }
    }
}

#[derive(Clone)]
pub struct TriangleMeshShape {
    pub concave_shape: ConcaveShape,
    pub local_aabb_min: Vec3A,
    pub local_aabb_max: Vec3A,
    pub mesh_interface: Arc<dyn StridingMeshInterface + Send + Sync>,
}

impl TriangleMeshShape {
    pub fn new(mesh_interface: Arc<dyn StridingMeshInterface + Send + Sync>) -> Self {
        let mut local_aabb_min = Vec3A::ZERO;
        let mut local_aabb_max = Vec3A::ZERO;
        let concave_shape = ConcaveShape::default();

        if mesh_interface.has_premade_aabb() {
            mesh_interface.get_premade_aabb(&mut local_aabb_min, &mut local_aabb_max);
        } else {
            Self::calc_local_aabb(
                &*mesh_interface,
                &mut local_aabb_min,
                &mut local_aabb_max,
                concave_shape.collision_margin,
            );
        }

        Self {
            concave_shape,
            local_aabb_min,
            local_aabb_max,
            mesh_interface,
        }
    }

    pub fn get_aabb(&self, trans: &Affine3A) -> (Vec3A, Vec3A) {
        let mut local_half_extents = 0.5 * (self.local_aabb_max - self.local_aabb_min);
        local_half_extents += Vec3A::splat(self.concave_shape.collision_margin);
        let local_center = 0.5 * (self.local_aabb_max + self.local_aabb_min);

        let abs_b = trans.matrix3.abs();
        let center = trans.transform_point3a(local_center);

        let extent = abs_b * local_half_extents;

        (center - extent, center + extent)
    }

    pub fn process_all_triangles(
        mesh_interface: &dyn StridingMeshInterface,
        callback: &mut dyn TriangleCallback,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
    ) {
        let mut filter_callback = FilteredCallback::new(callback, aabb_min, aabb_max);
        mesh_interface.internal_process_all_triangles(&mut filter_callback, &aabb_min, &aabb_max);
    }

    fn local_get_support_vertex(mesh_interface: &dyn StridingMeshInterface, vec: Vec3A) -> Vec3A {
        let mut support_callback = SupportVertexCallback::new(vec, Affine3A::IDENTITY);

        Self::process_all_triangles(
            mesh_interface,
            &mut support_callback,
            Vec3A::MIN,
            Vec3A::MAX,
        );

        support_callback.get_support_vertex_local()
    }

    fn calc_local_aabb(
        mesh_interface: &dyn StridingMeshInterface,
        local_aabb_min: &mut Vec3A,
        local_aabb_max: &mut Vec3A,
        collision_margin: f32,
    ) {
        for i in 0..3 {
            let mut vec = Vec3A::ZERO;

            vec[i] = 1.0;
            let mut tmp = Self::local_get_support_vertex(mesh_interface, vec);
            local_aabb_max[i] = tmp[i] + collision_margin;

            vec[i] = -1.0;
            tmp = Self::local_get_support_vertex(mesh_interface, vec);
            local_aabb_min[i] = tmp[i] - collision_margin;
        }
    }
}

struct SupportVertexCallback {
    support_vertex_local: Vec3A,
    world_trans: Affine3A,
    max_dot: f32,
    support_vec_local: Vec3A,
}

impl TriangleCallback for SupportVertexCallback {
    fn process_triangle(
        &mut self,
        triangle: &[Vec3A],
        _part_id: usize,
        _triangle_index: usize,
    ) -> bool {
        debug_assert_eq!(triangle.len(), 3);

        for vert in triangle {
            let dot = self.support_vec_local.dot(*vert);
            if dot > self.max_dot {
                self.max_dot = dot;
                self.support_vertex_local = *vert;
            }
        }

        true
    }
}

impl SupportVertexCallback {
    pub fn new(support_vec_world: Vec3A, world_trans: Affine3A) -> Self {
        Self {
            support_vertex_local: Vec3A::ZERO,
            world_trans,
            support_vec_local: world_trans.matrix3 * support_vec_world,
            max_dot: f32::MIN,
        }
    }

    pub const fn get_support_vertex_local(&self) -> Vec3A {
        self.support_vertex_local
    }
}
