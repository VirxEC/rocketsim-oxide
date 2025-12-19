use arrayvec::ArrayVec;
use glam::Vec3A;

use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        dispatch::collision_object_wrapper::CollisionObjectWrapper,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::{aabb_util_2::test_aabb_against_aabb, obb::Obb},
};

struct Hit {
    depth: f32,
    normal: Vec3A,
    axis_idx: usize,
}

/// Uses the Separating Axis Theorem (SAT)
///
/// - two convex polytopes intersect if no separating axis exists
/// - there are 15 axis
fn collide_obb_sat(a: &Obb, rhs: &Obb) -> Option<Hit> {
    let face_axes = [
        a.axis.x_axis,
        a.axis.y_axis,
        a.axis.z_axis,
        rhs.axis.x_axis,
        rhs.axis.y_axis,
        rhs.axis.z_axis,
    ];

    // Get smallest overlap
    let mut min_overlap = f32::MAX;
    let mut min_axis = Vec3A::ZERO;
    let mut min_axis_index = 0;

    let rpos = rhs.center - a.center;
    for (idx, axis) in face_axes.into_iter().enumerate() {
        // Project both boxes onto this axis
        let ra = a.project_obb_radius(axis);
        let rb = rhs.project_obb_radius(axis);

        // Project the center distance onto axis
        let distance = rpos.dot(axis).abs();

        // Compute overlap
        let overlap = ra + rb - distance;
        if overlap < 0.0 {
            // Found a separating axis — no collision
            return None;
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            min_axis = axis;
            min_axis_index = idx;
        }
    }

    // cross products (edge-edge)
    let mut axes_num = 5;
    for ai in &face_axes[0..3] {
        for &bj in &face_axes[3..6] {
            axes_num += 1;
            let Some(axis) = ai.cross(bj).try_normalize() else {
                // Parallel edges — skip this axis
                continue;
            };

            let ra = a.project_obb_radius(axis);
            let rb = rhs.project_obb_radius(axis);
            let distance = rpos.dot(axis).abs();

            let overlap = ra + rb - distance;
            if overlap < 0.0 {
                return None;
            }

            if overlap < min_overlap {
                min_overlap = overlap;
                min_axis = axis;
                min_axis_index = axes_num;
            }
        }
    }

    // At this point, we have an overlap on all axes -> collision
    // The axis with min overlap is our collision axis
    let mut normal = min_axis;
    // make normal point from A -> B
    if rpos.dot(min_axis) < 0.0 {
        normal = -normal;
    }

    Some(Hit {
        depth: min_overlap,
        normal,
        axis_idx: min_axis_index,
    })
}

/// Given a plane (normal, distance) and polygon (as Vec<Vec3A>), clip the polygon with the plane.
/// (Sutherland–Hodgman style)
fn clip_polygon_with_plane<const CAP: usize>(
    polygon: &ArrayVec<Vec3A, CAP>,
    plane_normal: Vec3A,
    plane_offset: f32,
) -> ArrayVec<Vec3A, CAP> {
    let mut out = ArrayVec::new();

    for (i, &a) in polygon.iter().enumerate() {
        let b = polygon[(i + 1) % polygon.len()];

        let da = plane_normal.dot(a) - plane_offset;
        let db = plane_normal.dot(b) - plane_offset;

        let inside_a = da <= 0.0;
        let inside_b = db <= 0.0;

        if inside_a && inside_b {
            out.push(b);
        } else if inside_a ^ inside_b {
            let t = da / (da - db);
            let i_pt = a + (b - a) * t;
            out.push(i_pt);

            if !inside_a {
                out.push(b);
            }
        }
    }

    out
}

pub struct ObbObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_0_obj: CollisionObjectWrapper<'a>,
    compound_1_obj: CollisionObjectWrapper<'a>,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ObbObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_0_obj: CollisionObjectWrapper<'a>,
        compound_1_obj: CollisionObjectWrapper<'a>,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_0_obj,
            compound_1_obj,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for ObbObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(self) -> Option<PersistentManifold> {
        let Some(CollisionShapes::Compound(compound_0_ref)) =
            self.compound_0_obj.object.get_collision_shape()
        else {
            unreachable!();
        };

        let Some(CollisionShapes::Compound(compound_1_ref)) =
            self.compound_1_obj.object.get_collision_shape()
        else {
            unreachable!();
        };

        let org_0_trans = self.compound_0_obj.object.get_world_transform();
        let aabb_0 = compound_0_ref.get_aabb(org_0_trans);
        let org_1_trans = self.compound_1_obj.object.get_world_transform();
        let aabb_1 = compound_1_ref.get_aabb(org_1_trans);
        if !test_aabb_against_aabb(&aabb_0, &aabb_1) {
            return None;
        }

        let child_0 = compound_0_ref.child.as_ref().unwrap();
        let child_0_trans = child_0.transform;
        let child_0_world_trans = org_0_trans * child_0_trans;

        let child_1 = compound_1_ref.child.as_ref().unwrap();
        let child_1_trans = child_1.transform;
        let child_1_world_trans = org_1_trans * child_1_trans;

        let obb0 = Obb::new(
            child_0_world_trans.translation,
            child_0_world_trans.matrix3,
            child_0.child_shape.get_half_extents(),
        );

        let obb1 = Obb::new(
            child_1_world_trans.translation,
            child_1_world_trans.matrix3,
            child_1.child_shape.get_half_extents(),
        );

        let hit = collide_obb_sat(&obb0, &obb1)?;
        let normal_on_b_in_world = child_1_trans.transform_vector3a(hit.normal);

        let mut manifold = PersistentManifold::new(
            self.compound_0_obj.object,
            self.compound_1_obj.object,
            false,
        );

        // solve for manifold points
        if hit.axis_idx > 5 {
            // edge-edge contact
            let extent = hit.normal * hit.depth * 0.5;
            let pt_on_a = obb0.closest_point_on_obb_edge(obb1.center + extent);
            let pt_on_b = obb1.closest_point_on_obb_edge(obb0.center - extent);
            let contact_point = (pt_on_a + pt_on_b) * 0.5;
            let depth = (pt_on_b - pt_on_a).dot(hit.normal);

            manifold.add_contact_point(
                self.compound_0_obj.object,
                self.compound_1_obj.object,
                normal_on_b_in_world,
                contact_point,
                depth,
                -1,
                -1,
                self.contact_added_callback,
            );

            manifold.refresh_contact_points(self.compound_0_obj.object, self.compound_1_obj.object);
            return Some(manifold);
        }

        let (ref_box, inc_box, ref_normal, axis_idx, side_sign) = if hit.axis_idx < 3 {
            (&obb0, &obb1, hit.normal, hit.axis_idx, 1.0)
        } else {
            (&obb1, &obb0, -hit.normal, hit.axis_idx - 3, -1.0)
        };

        let ref_face_verts = ref_box.get_face_verts(axis_idx, side_sign);
        let inc_face_verts = inc_box.find_face_verts(ref_normal);

        let mut clipped_polygon = ArrayVec::<Vec3A, 8>::new();
        clipped_polygon.extend(inc_face_verts);

        // clip against 4 side planes of ref face
        for (i, va) in ref_face_verts.into_iter().enumerate() {
            let vb = ref_face_verts[(i + 1) % 4];
            let edge = vb - va;
            let plane_normal = ref_normal.cross(edge).normalize();
            let plane_offset = plane_normal.dot(va);

            clipped_polygon = clip_polygon_with_plane(&clipped_polygon, plane_normal, plane_offset);
            if clipped_polygon.is_empty() {
                return None;
            }
        }

        for p in clipped_polygon {
            let depth = ref_normal.dot(p - ref_box.center) - ref_box.extent[axis_idx] * side_sign;
            if depth >= 0.0 {
                manifold.add_contact_point(
                    self.compound_0_obj.object,
                    self.compound_1_obj.object,
                    normal_on_b_in_world,
                    p,
                    depth,
                    -1,
                    -1,
                    self.contact_added_callback,
                );
            }
        }

        manifold.refresh_contact_points(self.compound_0_obj.object, self.compound_1_obj.object);
        Some(manifold)
    }
}
