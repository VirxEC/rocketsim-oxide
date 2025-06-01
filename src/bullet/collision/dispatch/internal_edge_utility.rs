use super::collision_object::CollisionObject;
use crate::bullet::{
    collision::{
        broadphase::quantized_bvh::{MAX_NUM_PARTS_IN_BITS, MyNodeOverlapCallback, QuantizedBvh},
        narrowphase::manifold_point::ManifoldPoint,
        shapes::{
            collision_shape::CollisionShapes,
            triangle_callback::TriangleCallback,
            triangle_info_map::{
                TRI_INFO_V0V1_CONVEX, TRI_INFO_V0V1_SWAP_NORMALB, TRI_INFO_V1V2_CONVEX,
                TRI_INFO_V1V2_SWAP_NORMALB, TRI_INFO_V2V0_CONVEX, TRI_INFO_V2V0_SWAP_NORMALB,
                TriangleInfoMap,
            },
            triangle_mesh::TriangleMesh,
            triangle_shape::TriangleShape,
        },
    },
    linear_math::AffineTranspose,
};
use glam::{Quat, Vec3, Vec3A};
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
    triangle_a: &'a TriangleShape,
    triangle_info_map: &'a mut TriangleInfoMap,
}

impl TriangleCallback for ConnectivityProcessor<'_> {
    fn process_triangle(
        &mut self,
        tri: &TriangleShape,
        _tri_aabb_min: Vec3A,
        _tri_aabb_max: Vec3A,
        part_id: usize,
        triangle_index: usize,
    ) -> bool {
        if self.part_id_a == part_id && self.triangle_index_a == triangle_index {
            return true;
        }

        if tri.normal_length < self.triangle_info_map.equal_vertex_threshold {
            return true;
        }

        if self.triangle_a.normal_length < self.triangle_info_map.equal_vertex_threshold {
            return true;
        }

        let mut num_shared = 0;
        let mut shared_verts_a = [0; 2];
        let mut shared_verts_b = [0; 2];

        for (i, vert_a) in self.triangle_a.points.into_iter().enumerate() {
            for (j, vert) in tri.points.into_iter().enumerate() {
                if vert_a.distance_squared(vert) < self.triangle_info_map.equal_vertex_threshold {
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

            let edge = (self.triangle_a.points[shared_verts_a[1]]
                - self.triangle_a.points[shared_verts_a[0]])
                .normalize();

            let other_index_b = 3 - (shared_verts_b[0] + shared_verts_b[1]);

            let mut edge_cross_a = edge.cross(self.triangle_a.normal).normalize();
            {
                let tmp = self.triangle_a.points[other_index_a]
                    - self.triangle_a.points[shared_verts_a[0]];
                if edge_cross_a.dot(tmp) < 0.0 {
                    edge_cross_a *= -1.0;
                }
            }

            let mut edge_cross_b = edge.cross(tri.normal).normalize();
            {
                let tmp = tri.points[other_index_b] - tri.points[shared_verts_b[0]];
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

                let dot_a = self.triangle_a.normal.dot(edge_cross_b);
                is_convex = dot_a < 0.0;
                corrected_angle = if is_convex { ang4 } else { -ang4 };
            }

            match sum_verts_a {
                1 => {
                    let edge = (-self.triangle_a.edges[0]).normalize();
                    let orn = Quat::from_axis_angle(edge.into(), -corrected_angle);
                    let mut computed_normal_b = orn * self.triangle_a.normal;
                    if computed_normal_b.dot(tri.normal) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TRI_INFO_V0V1_SWAP_NORMALB;
                    }

                    info.edge_v0_v1_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TRI_INFO_V0V1_CONVEX;
                    }
                }
                2 => {
                    let edge = (-self.triangle_a.edges[2]).normalize();
                    let orn = Quat::from_axis_angle(edge.into(), -corrected_angle);
                    let mut computed_normal_b = orn * self.triangle_a.normal;
                    if computed_normal_b.dot(tri.normal) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TRI_INFO_V2V0_SWAP_NORMALB;
                    }

                    info.edge_v2_v0_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TRI_INFO_V2V0_CONVEX;
                    }
                }
                3 => {
                    let edge = (-self.triangle_a.edges[1]).normalize();
                    let orn = Quat::from_axis_angle(edge.into(), -corrected_angle);
                    let mut computed_normal_b = orn * self.triangle_a.normal;
                    if computed_normal_b.dot(tri.normal) < 0.0 {
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
    mesh_interface: &TriangleMesh,
) -> TriangleInfoMap {
    let mut triangle_info_map = TriangleInfoMap::default();
    triangle_info_map
        .internal_map
        .reserve(mesh_interface.get_total_num_faces());

    let part_id = 0;
    let (tris, aabbs) = mesh_interface.get_tris_aabbs(part_id);

    for (i, (triangle_a, (aabb_min, aabb_max))) in tris.iter().zip(aabbs).enumerate() {
        let mut connectivity_processor = ConnectivityProcessor {
            part_id_a: part_id,
            triangle_index_a: i,
            triangle_a,
            triangle_info_map: &mut triangle_info_map,
        };

        let mut my_node_callback =
            MyNodeOverlapCallback::new(mesh_interface, &mut connectivity_processor);
        quantized_bvh.report_aabb_overlapping_node(&mut my_node_callback, *aabb_min, *aabb_max);
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
    let edge_cross = edge.cross(tri_normal_org).normalize();
    let cur_angle = get_angle(edge_cross, tri_normal_org, local_contact_normal_on_b);

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
    let tri = tri_mesh.get_mesh_interface().get_triangle(part_id, index);
    let nearest = nearst_point_in_line_segment(cp.local_point_b, tri.points[0], tri.points[1]);
    let contact = cp.local_point_b;

    let local_contact_normal_on_b =
        tri_mesh_col_obj.get_world_transform().matrix3.transpose() * cp.normal_world_on_b;
    debug_assert!(local_contact_normal_on_b.is_normalized());

    let mut best_edge = None;
    let mut dists = Vec3::MAX;
    let mut dist_to_best_edge = f32::MAX;

    if info.edge_v0_v1_angle.abs() < info_map.max_edge_angle_threshold {
        dists.x = (contact - nearest).length();
        if dists.x < dist_to_best_edge {
            best_edge = Some(0);
            dist_to_best_edge = dists.x;
        }
    }

    if info.edge_v1_v2_angle.abs() < info_map.max_edge_angle_threshold {
        let nearest = nearst_point_in_line_segment(cp.local_point_b, tri.points[1], tri.points[2]);
        dists.y = (contact - nearest).length();
        if dists.y < dist_to_best_edge {
            best_edge = Some(1);
            dist_to_best_edge = dists.y;
        }
    }

    if info.edge_v2_v0_angle.abs() < info_map.max_edge_angle_threshold {
        let nearest = nearst_point_in_line_segment(cp.local_point_b, tri.points[2], tri.points[0]);
        dists.z = (contact - nearest).length();
        if dists.z < dist_to_best_edge {
            best_edge = Some(2);
        }
    }

    match best_edge {
        Some(0)
            if info.edge_v0_v1_angle.abs() < info_map.max_edge_angle_threshold
                && dists.x < info_map.edge_distance_threshold =>
        {
            if info.edge_v0_v1_angle != 0.0 {
                let is_edge_convex = info.flags & TRI_INFO_V0V1_CONVEX != 0;
                let swap_factor = if is_edge_convex { 1.0 } else { -1.0 };

                let edge = -tri.edges[0];
                let n_a = swap_factor * tri.normal;
                let orn = Quat::from_axis_angle(edge.into(), info.edge_v0_v1_angle);
                let mut computed_normal_b = orn * tri.normal;
                if info.flags & TRI_INFO_V0V1_SWAP_NORMALB != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal =
                    n_dot_a < info_map.convex_epsilon && n_dot_b < info_map.convex_epsilon;

                if !back_facing_normal {
                    if let Some(clamped_local_normal) = clamp_normal(
                        edge,
                        swap_factor * tri.normal,
                        local_contact_normal_on_b,
                        info.edge_v0_v1_angle,
                    ) {
                        if clamped_local_normal.dot(front_facing * tri.normal) > 0.0 {
                            let new_normal = tri_mesh_col_obj.get_world_transform().matrix3
                                * clamped_local_normal;
                            cp.normal_world_on_b = new_normal;
                            cp.position_world_on_b =
                                cp.position_world_on_a - new_normal * cp.distance_1;
                            cp.local_point_b = tri_mesh_col_obj
                                .get_world_transform()
                                .inv_x_form(cp.position_world_on_b);
                        }
                    }
                    return;
                }
            }
        }
        Some(1)
            if info.edge_v1_v2_angle.abs() < info_map.max_edge_angle_threshold
                && dists.y < info_map.edge_distance_threshold =>
        {
            if info.edge_v1_v2_angle != 0.0 {
                let is_edge_convex = info.flags & TRI_INFO_V1V2_CONVEX != 0;
                let swap_factor = if is_edge_convex { 1.0 } else { -1.0 };

                let edge = -tri.edges[1];
                let n_a = swap_factor * tri.normal;
                let orn = Quat::from_axis_angle(edge.into(), info.edge_v1_v2_angle);
                let mut computed_normal_b = orn * tri.normal;
                if info.flags & TRI_INFO_V1V2_SWAP_NORMALB != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal =
                    n_dot_a < info_map.convex_epsilon && n_dot_b < info_map.convex_epsilon;

                if !back_facing_normal {
                    if let Some(clamped_local_normal) = clamp_normal(
                        edge,
                        swap_factor * tri.normal,
                        local_contact_normal_on_b,
                        info.edge_v1_v2_angle,
                    ) {
                        if clamped_local_normal.dot(front_facing * tri.normal) > 0.0 {
                            let new_normal = tri_mesh_col_obj.get_world_transform().matrix3
                                * clamped_local_normal;
                            cp.normal_world_on_b = new_normal;
                            cp.position_world_on_b =
                                cp.position_world_on_a - new_normal * cp.distance_1;
                            cp.local_point_b = tri_mesh_col_obj
                                .get_world_transform()
                                .inv_x_form(cp.position_world_on_b);
                        }
                    }
                    return;
                }
            }
        }
        Some(2)
            if info.edge_v2_v0_angle.abs() < info_map.max_edge_angle_threshold
                && dists.z < info_map.edge_distance_threshold =>
        {
            if info.edge_v2_v0_angle != 0.0 {
                let is_edge_convex = info.flags & TRI_INFO_V2V0_CONVEX != 0;
                let swap_factor = if is_edge_convex { 1.0 } else { -1.0 };

                let edge = -tri.edges[2];
                let n_a = swap_factor * tri.normal;
                let orn = Quat::from_axis_angle(edge.into(), info.edge_v2_v0_angle);
                let mut computed_normal_b = orn * tri.normal;
                if info.flags & TRI_INFO_V2V0_SWAP_NORMALB != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal =
                    n_dot_a < info_map.convex_epsilon && n_dot_b < info_map.convex_epsilon;

                if !back_facing_normal {
                    if let Some(clamped_local_normal) = clamp_normal(
                        edge,
                        swap_factor * tri.normal,
                        local_contact_normal_on_b,
                        info.edge_v2_v0_angle,
                    ) {
                        if clamped_local_normal.dot(front_facing * tri.normal) > 0.0 {
                            let new_normal = tri_mesh_col_obj.get_world_transform().matrix3
                                * clamped_local_normal;
                            cp.normal_world_on_b = new_normal;
                            cp.position_world_on_b =
                                cp.position_world_on_a - new_normal * cp.distance_1;
                            cp.local_point_b = tri_mesh_col_obj
                                .get_world_transform()
                                .inv_x_form(cp.position_world_on_b);
                        }
                    }
                    return;
                }
            }
        }
        _ => return,
    }

    let new_normal = tri.normal * front_facing;
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
