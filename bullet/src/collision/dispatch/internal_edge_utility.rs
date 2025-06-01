use super::collision_object::CollisionObject;
use crate::{
    collision::{
        broadphase::quantized_bvh::{MAX_NUM_PARTS_IN_BITS, MyNodeOverlapCallback, QuantizedBvh},
        narrowphase::manifold_point::ManifoldPoint,
        shapes::{
            collision_shape::CollisionShapes,
            striding_mesh_interface::StridingMeshInterface,
            triangle_callback::TriangleCallback,
            triangle_info_map::{
                TRI_INFO_V0V1_CONVEX, TRI_INFO_V0V1_SWAP_NORMALB, TRI_INFO_V1V2_CONVEX,
                TRI_INFO_V1V2_SWAP_NORMALB, TRI_INFO_V2V0_CONVEX, TRI_INFO_V2V0_SWAP_NORMALB,
                TriangleInfoMap,
            },
            triangle_shape::TriangleShape,
        },
    },
    linear_math::AffineTranspose,
};
use glam::{Quat, Vec3A};
use std::{f32::consts::PI, mem};

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
    fn process_triangle(
        &mut self,
        triangle: &[Vec3A],
        _tri_aabb_min: Vec3A,
        _tri_aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool {
        // todo: see if we can use the triangle aabbs to speed up load times
        if self.part_id_a == part_id && self.triangle_index_a == triangle_index {
            return true;
        }

        let cross_b_sqr = (triangle[1] - triangle[0])
            .cross(triangle[2] - triangle[0])
            .length_squared();
        if cross_b_sqr < self.triangle_info_map.equal_vertex_threshold {
            return true;
        }

        let cross_a_sqr = (self.triangle_vertices_a[1] - self.triangle_vertices_a[0])
            .cross(self.triangle_vertices_a[2] - self.triangle_vertices_a[0])
            .length_squared();
        if cross_a_sqr < self.triangle_info_map.equal_vertex_threshold {
            return true;
        }

        let mut num_shared = 0;
        let mut shared_verts_a = [0; 2];
        let mut shared_verts_b = [0; 2];

        for (i, vert_a) in self.triangle_vertices_a.iter().enumerate() {
            for (j, vert) in triangle.iter().enumerate() {
                if vert_a.distance_squared(*vert) < self.triangle_info_map.equal_vertex_threshold {
                    debug_assert!(num_shared < 2, "degenerate triangle");

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

                let [a, b] = unsafe { shared_verts_b.get_disjoint_unchecked_mut([0, 1]) };
                mem::swap(a, b);
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

        true
    }
}

pub fn generate_internal_edge_info(
    quantized_bvh: &QuantizedBvh,
    mesh_interface: &dyn StridingMeshInterface,
) -> TriangleInfoMap {
    let mesh_scaling = mesh_interface.get_scaling();

    let mut triangle_info_map = TriangleInfoMap::default();
    triangle_info_map
        .internal_map
        .reserve(mesh_interface.get_total_num_faces());

    let mut triangle = [Vec3A::ZERO; 3];

    for part_id in 0..mesh_interface.get_num_sub_parts() {
        let (verts, ids, aabbs) = mesh_interface.get_verts_ids_aabbs(part_id);

        for (i, (inner_ids, (aabb_min, aabb_max))) in ids.chunks_exact(3).zip(aabbs).enumerate() {
            for (vert, &id) in triangle.iter_mut().zip(inner_ids).rev() {
                *vert = verts[id] * mesh_scaling;
            }

            let mut connectivity_processor = ConnectivityProcessor {
                part_id_a: part_id,
                triangle_index_a: i,
                triangle_vertices_a: &triangle,
                triangle_info_map: &mut triangle_info_map,
            };

            let mut my_node_callback =
                MyNodeOverlapCallback::new(mesh_interface, &mut connectivity_processor);
            quantized_bvh.report_aabb_overlapping_node(&mut my_node_callback, *aabb_min, *aabb_max);
        }
    }

    triangle_info_map
}

fn nearst_point_in_line_segment(point: Vec3A, line0: Vec3A, line1: Vec3A) -> Vec3A {
    let line_delta = line1 - line0;

    if line_delta.length_squared() < f32::EPSILON * f32::EPSILON {
        line0
    } else {
        let delta = (point - line0).dot(line_delta) / line_delta.dot(line_delta);
        line0 + line_delta * delta.clamp(0.0, 1.0)
    }
}

fn clamp_normal(
    edge: Vec3A,
    tri_normal_org: Vec3A,
    local_contact_normal_on_b: Vec3A,
    corrected_edge_angle: f32,
) -> Option<Vec3A> {
    let tri_normal = tri_normal_org;

    let edge_cross = edge.cross(tri_normal).normalize();
    let cur_angle = get_angle(edge_cross, tri_normal, local_contact_normal_on_b);

    if (corrected_edge_angle < 0.0 && cur_angle < corrected_edge_angle)
        || (corrected_edge_angle >= 0.0 && cur_angle > corrected_edge_angle)
    {
        let diff_angle = corrected_edge_angle - cur_angle;
        let rotation = Quat::from_axis_angle(edge.into(), diff_angle);
        Some(rotation * local_contact_normal_on_b)
    } else {
        None
    }
}

pub fn adjust_internal_edge_contacts(
    cp: &mut ManifoldPoint,
    tri_mesh_col_obj: &CollisionObject,
    _other_col_obj: &CollisionObject,
    part_id: usize,
    index: usize,
) {
    let tri_col_shape = tri_mesh_col_obj.get_collision_shape().unwrap().borrow();
    let CollisionShapes::TriangleMesh(tri_mesh) = &*tri_col_shape else {
        return;
    };

    let info_map = tri_mesh.get_triangle_info_map();

    let hash = get_hash(part_id, index);
    let info = info_map.internal_map.get(&hash).unwrap();

    let front_facing = 1.0;
    let verts = tri_mesh.get_mesh_interface().get_triangle(part_id, index);
    let tri_normal = (verts[1] - verts[0]).cross(verts[2] - verts[0]).normalize();
    let nearest = nearst_point_in_line_segment(cp.local_point_b, verts[0], verts[1]);
    let contact = cp.local_point_b;

    let mut is_near_edge = false;
    let mut num_concave_edge_hits = 0;

    let local_contact_normal_on_b =
        tri_mesh_col_obj.get_world_transform().matrix3.transpose() * cp.normal_world_on_b;
    debug_assert!(local_contact_normal_on_b.is_normalized());

    let mut best_edge = None;
    let mut dist_to_best_edge = f32::MAX;

    if info.edge_v0_v1_angle.abs() < info_map.max_edge_angle_threshold {
        let len = (contact - nearest).length();
        if len < dist_to_best_edge {
            best_edge = Some(0);
            dist_to_best_edge = len;
        }
    }

    if info.edge_v1_v2_angle.abs() < info_map.max_edge_angle_threshold {
        let nearest = nearst_point_in_line_segment(cp.local_point_b, verts[1], verts[2]);
        let len = (contact - nearest).length();
        if len < dist_to_best_edge {
            best_edge = Some(1);
            dist_to_best_edge = len;
        }
    }

    if info.edge_v2_v0_angle.abs() < info_map.max_edge_angle_threshold {
        let nearest = nearst_point_in_line_segment(cp.local_point_b, verts[2], verts[0]);
        let len = (contact - nearest).length();
        if len < dist_to_best_edge {
            best_edge = Some(2);
            // dist_to_best_edge = len;
        }
    }

    let Some(best_edge) = best_edge else {
        return;
    };

    if best_edge == 0 && info.edge_v0_v1_angle.abs() < info_map.max_edge_angle_threshold {
        let len = (contact - nearest).length();
        if len < info_map.edge_distance_threshold {
            let edge = verts[0] - verts[1];
            is_near_edge = true;

            if info.edge_v0_v1_angle == 0.0 {
                num_concave_edge_hits += 1;
            } else {
                let is_edge_convex = info.flags & TRI_INFO_V0V1_CONVEX != 0;
                let swap_factor = if is_edge_convex { 1.0 } else { -1.0 };

                let n_a = swap_factor * tri_normal;
                let orn = Quat::from_axis_angle(edge.into(), info.edge_v0_v1_angle);
                let mut computed_normal_b = orn * tri_normal;
                if info.flags & TRI_INFO_V0V1_SWAP_NORMALB != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal =
                    n_dot_a < info_map.convex_epsilon && n_dot_b < info_map.convex_epsilon;

                if back_facing_normal {
                    num_concave_edge_hits += 1;
                } else if let Some(clamped_local_normal) = clamp_normal(
                    edge,
                    swap_factor * tri_normal,
                    local_contact_normal_on_b,
                    info.edge_v0_v1_angle,
                ) {
                    if clamped_local_normal.dot(front_facing * tri_normal) > 0.0 {
                        let new_normal =
                            tri_mesh_col_obj.get_world_transform().matrix3 * clamped_local_normal;
                        cp.normal_world_on_b = new_normal;
                        cp.position_world_on_b =
                            cp.position_world_on_a - new_normal * cp.distance_1;
                        cp.local_point_b = tri_mesh_col_obj
                            .get_world_transform()
                            .inv_x_form(cp.position_world_on_b);
                    }
                }
            }
        }
    }

    if best_edge == 1 && info.edge_v1_v2_angle.abs() < info_map.max_edge_angle_threshold {
        let nearest = nearst_point_in_line_segment(contact, verts[1], verts[2]);
        let len = (contact - nearest).length();
        if len < info_map.edge_distance_threshold {
            is_near_edge = true;
            let edge = verts[1] - verts[2];

            if info.edge_v1_v2_angle == 0.0 {
                num_concave_edge_hits += 1;
            } else {
                let is_edge_convex = info.flags & TRI_INFO_V1V2_CONVEX != 0;
                let swap_factor = if is_edge_convex { 1.0 } else { -1.0 };

                let n_a = swap_factor * tri_normal;
                let orn = Quat::from_axis_angle(edge.into(), info.edge_v1_v2_angle);
                let mut computed_normal_b = orn * tri_normal;
                if info.flags & TRI_INFO_V1V2_SWAP_NORMALB != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal =
                    n_dot_a < info_map.convex_epsilon && n_dot_b < info_map.convex_epsilon;

                if back_facing_normal {
                    num_concave_edge_hits += 1;
                } else if let Some(clamped_local_normal) = clamp_normal(
                    edge,
                    swap_factor * tri_normal,
                    local_contact_normal_on_b,
                    info.edge_v1_v2_angle,
                ) {
                    if clamped_local_normal.dot(front_facing * tri_normal) > 0.0 {
                        let new_normal =
                            tri_mesh_col_obj.get_world_transform().matrix3 * clamped_local_normal;
                        cp.normal_world_on_b = new_normal;
                        cp.position_world_on_b =
                            cp.position_world_on_a - new_normal * cp.distance_1;
                        cp.local_point_b = tri_mesh_col_obj
                            .get_world_transform()
                            .inv_x_form(cp.position_world_on_b);
                    }
                }
            }
        }
    }

    if best_edge == 2 && info.edge_v2_v0_angle.abs() < info_map.max_edge_angle_threshold {
        let nearest = nearst_point_in_line_segment(contact, verts[2], verts[0]);
        let len = (contact - nearest).length();
        if len < info_map.edge_distance_threshold {
            is_near_edge = true;
            let edge = verts[2] - verts[0];

            if info.edge_v2_v0_angle == 0.0 {
                num_concave_edge_hits += 1;
            } else {
                let is_edge_convex = info.flags & TRI_INFO_V2V0_CONVEX != 0;
                let swap_factor = if is_edge_convex { 1.0 } else { -1.0 };

                let n_a = swap_factor * tri_normal;
                let orn = Quat::from_axis_angle(edge.into(), info.edge_v2_v0_angle);
                let mut computed_normal_b = orn * tri_normal;
                if info.flags & TRI_INFO_V2V0_SWAP_NORMALB != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal =
                    n_dot_a < info_map.convex_epsilon && n_dot_b < info_map.convex_epsilon;

                if back_facing_normal {
                    num_concave_edge_hits += 1;
                } else if let Some(clamped_local_normal) = clamp_normal(
                    edge,
                    swap_factor * tri_normal,
                    local_contact_normal_on_b,
                    info.edge_v2_v0_angle,
                ) {
                    if clamped_local_normal.dot(front_facing * tri_normal) > 0.0 {
                        let new_normal =
                            tri_mesh_col_obj.get_world_transform().matrix3 * clamped_local_normal;
                        cp.normal_world_on_b = new_normal;
                        cp.position_world_on_b =
                            cp.position_world_on_a - new_normal * cp.distance_1;
                        cp.local_point_b = tri_mesh_col_obj
                            .get_world_transform()
                            .inv_x_form(cp.position_world_on_b);
                    }
                }
            }
        }
    }

    if !is_near_edge || num_concave_edge_hits == 0 {
        return;
    }

    let new_normal = tri_normal * front_facing;
    let d = new_normal.dot(local_contact_normal_on_b);
    if d < 0.0 {
        return;
    }

    cp.normal_world_on_b = tri_mesh_col_obj.get_world_transform().matrix3 * new_normal;
    cp.position_world_on_b = cp.position_world_on_a - cp.normal_world_on_b * cp.distance_1;
    cp.local_point_b = tri_mesh_col_obj
        .get_world_transform()
        .inv_x_form(cp.position_world_on_b);
}
