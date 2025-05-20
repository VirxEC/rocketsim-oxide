use crate::collision::{
    broadphase::quantized_bvh::MAX_NUM_PARTS_IN_BITS,
    shapes::{
        bvh_triangle_mesh_shape::BvhTriangleMeshShape,
        striding_mesh_interface::StridingMeshInterface,
        triangle_callback::TriangleCallback,
        triangle_info_map::{
            TRI_INFO_V0V1_CONVEX, TRI_INFO_V0V1_SWAP_NORMALB, TRI_INFO_V1V2_CONVEX,
            TRI_INFO_V1V2_SWAP_NORMALB, TRI_INFO_V2V0_CONVEX, TRI_INFO_V2V0_SWAP_NORMALB,
            TriangleInfoMap,
        },
        triangle_shape::TriangleShape,
    },
};
use glam::{Quat, Vec3A};
use std::{f32::consts::PI, mem, ptr};

// pub(crate) enum InternalEdgeAdjustFlags {
//     TriangleConvexBackfaceMode = 1,
//     TriangleConcaveDoubleSided = 2,
//     TriangleConvexDoubleSided = 4,
// }

const fn get_hash(part_id: usize, triangle_index: usize) -> i32 {
    ((part_id as i32) << (31 - MAX_NUM_PARTS_IN_BITS as i32)) | triangle_index as i32
}

fn get_angle(edge_a: Vec3A, normal_a: Vec3A, normal_b: Vec3A) -> f32 {
    normal_b.dot(edge_a).atan2(normal_b.dot(normal_a))
}

struct ConnectivityProcessor<'a> {
    part_id_a: usize,
    triangle_index_a: usize,
    triangle_vertices_a: &'a [Vec3A; 3],
    triangle_info_map: &'a mut TriangleInfoMap,
}

impl TriangleCallback for ConnectivityProcessor<'_> {
    fn process_triangle(&mut self, triangle: &[Vec3A], part_id: usize, triangle_index: usize) {
        if self.part_id_a == part_id && self.triangle_index_a == triangle_index {
            return;
        }

        let cross_b_sqr = (triangle[1] - triangle[0])
            .cross(triangle[2] - triangle[0])
            .length_squared();
        if cross_b_sqr < self.triangle_info_map.equal_vertex_threshold {
            return;
        }

        let cross_a_sqr = (self.triangle_vertices_a[1] - self.triangle_vertices_a[0])
            .cross(self.triangle_vertices_a[2] - self.triangle_vertices_a[0])
            .length_squared();
        if cross_a_sqr < self.triangle_info_map.equal_vertex_threshold {
            return;
        }

        let mut num_shared = 0;
        let mut shared_verts_a = [0; 2];
        let mut shared_verts_b = [0; 2];

        for (i, vert_a) in self.triangle_vertices_a.iter().enumerate() {
            for (j, vert) in triangle.iter().enumerate() {
                if vert_a.distance_squared(*vert) < self.triangle_info_map.equal_vertex_threshold {
                    debug_assert!(num_shared < 3, "degenerate triangle");

                    shared_verts_a[num_shared] = i;
                    shared_verts_b[num_shared] = j;
                    num_shared += 1;
                }
            }
        }

        if num_shared == 2 {
            if shared_verts_a[0] == 0 && shared_verts_a[1] == 2 {
                shared_verts_a[0] = 2;
                shared_verts_a[1] = 0;

                let (a, b) = shared_verts_b.split_at_mut(1);
                mem::swap(&mut a[0], &mut b[0]);
            }

            let hash = get_hash(self.part_id_a, self.triangle_index_a);
            let info = self.triangle_info_map.internal_map.entry(hash).or_default();

            let sum_verts_a = shared_verts_a[0] + shared_verts_a[1];
            let other_index_a = 3 - sum_verts_a;

            let edge = (self.triangle_vertices_a[shared_verts_a[1]]
                - self.triangle_vertices_a[shared_verts_a[0]])
                .normalize();

            let tri_a = TriangleShape::new(*self.triangle_vertices_a);

            let other_index_b = 3 - (shared_verts_b[0] + shared_verts_b[1]);
            let tri_b = TriangleShape::new([
                triangle[shared_verts_b[1]],
                triangle[shared_verts_b[0]],
                triangle[other_index_b],
            ]);

            let normal_a = tri_a.calc_normal();
            let normal_b = tri_b.calc_normal();

            let mut edge_cross_a = edge.cross(normal_a).normalize();

            {
                let tmp = self.triangle_vertices_a[other_index_a]
                    - self.triangle_vertices_a[shared_verts_a[0]];
                if edge_cross_a.dot(tmp) < 0.0 {
                    edge_cross_a *= -1.0;
                }
            }

            let mut edge_cross_b = edge.cross(normal_b).normalize();

            {
                let tmp = triangle[other_index_b] - triangle[shared_verts_b[0]];
                if edge_cross_b.dot(tmp) < 0.0 {
                    edge_cross_b *= -1.0;
                }
            }

            let calculated_edge = edge_cross_a.cross(edge_cross_b);
            let len2 = calculated_edge.length_squared();

            let mut corrected_angle = 0.0;
            let mut is_convex = false;

            if len2 >= self.triangle_info_map.planar_epsilon {
                let calculated_edge = calculated_edge.normalize();
                let calculated_normal_a = calculated_edge.cross(edge_cross_a).normalize();
                let angle2 = get_angle(calculated_normal_a, edge_cross_a, edge_cross_b);
                let ang4 = PI - angle2;

                let dot_a = normal_a.dot(edge_cross_b);
                is_convex = dot_a < 0.0;
                corrected_angle = if is_convex { ang4 } else { -ang4 };
            }

            match sum_verts_a {
                1 => {
                    let edge =
                        (self.triangle_vertices_a[0] - self.triangle_vertices_a[1]).normalize();
                    let orn = Quat::from_axis_angle(edge.into(), -corrected_angle);
                    let mut computed_normal_b = orn * normal_a;
                    if computed_normal_b.dot(normal_b) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TRI_INFO_V0V1_SWAP_NORMALB;
                    }

                    info.edge_v0_v1_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TRI_INFO_V0V1_CONVEX;
                    }
                }
                2 => {
                    let edge =
                        (self.triangle_vertices_a[2] - self.triangle_vertices_a[0]).normalize();
                    let orn = Quat::from_axis_angle(edge.into(), -corrected_angle);
                    let mut computed_normal_b = orn * normal_a;
                    if computed_normal_b.dot(normal_b) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TRI_INFO_V2V0_SWAP_NORMALB;
                    }

                    info.edge_v2_v0_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TRI_INFO_V2V0_CONVEX;
                    }
                }
                3 => {
                    let edge =
                        (self.triangle_vertices_a[1] - self.triangle_vertices_a[2]).normalize();
                    let orn = Quat::from_axis_angle(edge.into(), -corrected_angle);
                    let mut computed_normal_b = orn * normal_a;
                    if computed_normal_b.dot(normal_b) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TRI_INFO_V1V2_SWAP_NORMALB;
                    }

                    info.edge_v1_v2_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TRI_INFO_V1V2_CONVEX;
                    }
                }
                _ => unreachable!(),
            }
        }
    }
}

pub fn generate_internal_edge_info(
    tri_mesh_shape: &mut BvhTriangleMeshShape,
    triangle_info_map: &mut TriangleInfoMap,
) {
    if tri_mesh_shape.get_triangle_info_map().is_some() {
        return;
    }

    let mesh_interface: &dyn StridingMeshInterface = tri_mesh_shape.get_mesh_interface();
    let mesh_scaling = mesh_interface.get_scaling();

    triangle_info_map
        .internal_map
        .reserve(mesh_interface.get_total_num_faces());

    for part_id in 0..mesh_interface.get_num_sub_parts() {
        let mut vertex_base = ptr::null();
        let mut num_verts = 0;
        let mut vertex_stride = 0;
        let mut index_base = ptr::null();
        let mut num_faces = 0;
        let mut index_stride = 0;

        let mut triangle_verts = [Vec3A::ZERO; 3];

        mesh_interface.get_locked_read_only_vertex_index_base(
            &mut vertex_base,
            &mut num_verts,
            &mut vertex_stride,
            &mut index_base,
            &mut index_stride,
            &mut num_faces,
            part_id,
        );

        for gfx_index in 0..num_faces {
            let tri_indices =
                unsafe { index_base.byte_add(gfx_index * index_stride) }.cast::<u32>();

            for (i, vert) in triangle_verts.iter_mut().enumerate().rev() {
                let graphics_base =
                    unsafe { vertex_base.byte_add(*tri_indices.add(i) as usize * vertex_stride) }
                        .cast::<f32>();

                vert.x = unsafe { *graphics_base.add(0) };
                vert.y = unsafe { *graphics_base.add(1) };
                vert.z = unsafe { *graphics_base.add(2) };
                *vert *= mesh_scaling;
            }

            let aabb_min = triangle_verts[0]
                .min(triangle_verts[1])
                .min(triangle_verts[2]);
            let aabb_max = triangle_verts[0]
                .max(triangle_verts[1])
                .max(triangle_verts[2]);

            let mut connectivity_processor = ConnectivityProcessor {
                part_id_a: part_id,
                triangle_index_a: gfx_index,
                triangle_vertices_a: &triangle_verts,
                triangle_info_map,
            };

            tri_mesh_shape.process_all_triangles(&mut connectivity_processor, aabb_min, aabb_max);
        }
    }
}
