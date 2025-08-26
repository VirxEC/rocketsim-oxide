use std::f32;

use crate::bullet::{
    collision::{
        broadphase::collision_algorithm::CollisionAlgorithm,
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
        },
        narrowphase::persistent_manifold::{
            ContactAddedCallback, MANIFOLD_CACHE_SIZE, PersistentManifold,
        },
        shapes::collision_shape::CollisionShapes,
    },
    linear_math::{AffineExt, aabb_util_2::test_aabb_against_aabb},
};
use glam::{Mat3A, Vec3A};

struct Obb {
    center: Vec3A,
    axis: Mat3A,
    extent: Vec3A,
}

struct Aabb {
    center: Vec3A,
    extent: Vec3A,
}

impl From<Aabb> for Obb {
    #[inline]
    fn from(value: Aabb) -> Self {
        Self {
            center: value.center,
            extent: value.extent,
            axis: Mat3A::IDENTITY,
        }
    }
}

struct Hit {
    depth: f32,
    normal: Vec3A,
    axis_type: u8,
    obb_axis: Option<u8>,
    aabb_axis: Option<u8>,
}

/// Uses the Separating Axis Theorem (SAT)
///
/// - two convex polytopes intersect if no separating axis exists
/// - there are 15 axis
fn collide_obb_aabb_sat(a: &Obb, b: &Aabb) -> Option<Hit> {
    const B_AXIS: [Vec3A; 3] = [Vec3A::X, Vec3A::Y, Vec3A::Z];

    let r = [a.axis.x_axis, a.axis.y_axis, a.axis.z_axis];

    // tW/tA
    let t_w = b.center - a.center;
    let t_a = a.axis.transpose() * t_w;

    // avoid false separations due to near-parallel axes by adding EPS
    let abs_r = [
        r[0].abs() + f32::EPSILON,
        r[1].abs() + f32::EPSILON,
        r[2].abs() + f32::EPSILON,
    ];

    let mut depth = f32::INFINITY;
    let mut best_axis_w = Vec3A::ZERO;
    let mut best_axis_type = 0;
    let mut best_obb_axis = None;
    let mut best_aabb_axis = None;

    let mut update = |dist: f32,
                      ra: f32,
                      rb: f32,
                      l_w: Vec3A,
                      orient: bool,
                      axis_type: u8,
                      obb_axis: Option<u8>,
                      aabb_axis: Option<u8>| {
        let overlap = (ra + rb) - dist;
        if overlap < 0.0 {
            return false;
        }

        if overlap < depth {
            depth = overlap;
            best_axis_w = (f32::from(orient) * 2.0 - 1.0) * l_w;
            best_axis_type = axis_type;
            best_obb_axis = obb_axis;
            best_aabb_axis = aabb_axis;
        }

        true
    };

    for axis in 0..3 {
        let i = usize::from(axis);
        let dist = t_a[i].abs();
        let ra = a.extent[i];
        let rb = b.extent.dot(abs_r[i]);
        let l_w = r[i];

        if !update(dist, ra, rb, l_w, t_a[i] >= 0.0, axis, Some(axis), None) {
            return None;
        }
    }

    {
        let ra = Mat3A::from_cols(abs_r[0], abs_r[1], abs_r[2]) * a.extent;

        let tw_abs = t_w.abs();
        let orient: [bool; 3] = t_w.cmpge(Vec3A::ZERO).into();

        for axis in 0..3 {
            let j = usize::from(axis);
            if !update(
                tw_abs[j],
                ra[j],
                b.extent[j],
                Vec3A::X,
                orient[j],
                3 + axis,
                None,
                Some(axis),
            ) {
                return None;
            }
        }
    }

    let mut test_cross = |l_w: Vec3A,
                          ra: f32,
                          rb: f32,
                          axis_type: u8,
                          obb_axis: Option<u8>,
                          aabb_axis: Option<u8>| {
        let Some(l) = l_w.try_normalize() else {
            return true;
        };

        let align = t_w.dot(l);
        let dist = align.abs();
        let orient = align >= 0.0;
        update(dist, ra, rb, l, orient, axis_type, obb_axis, aabb_axis)
    };

    for obb_axis in 0..3 {
        let i = usize::from(obb_axis);
        for aabb_axis in 0..3 {
            let j = usize::from(aabb_axis);
            let l_w = r[i].cross(B_AXIS[j]);
            let ra = a.extent[(i + 1) % 3] * abs_r[(i + 2) % 3][j]
                + a.extent[(i + 2) % 3] * abs_r[(i + 1) % 3][j];
            let rb = b.extent[(j + 1) % 3] * abs_r[i][(j + 2) % 3]
                + b.extent[(j + 2) % 3] * abs_r[i][(j + 1) % 3];

            if !test_cross(
                l_w,
                ra,
                rb,
                6 + obb_axis * 3 + aabb_axis,
                Some(obb_axis),
                Some(aabb_axis),
            ) {
                return None;
            }
        }
    }

    Some(Hit {
        depth,
        normal: best_axis_w.normalize(),
        axis_type: best_axis_type,
        obb_axis: best_obb_axis,
        aabb_axis: best_aabb_axis,
    })
}

/// Returns the 4 corners of the face on the `axis_idx` (0 = X, 1 = Y, 2 = Z)
/// and `side_sign` = +1 or -1
fn get_face_verts(b: &Obb, axis_idx: usize, side_sign: f32) -> [Vec3A; 4] {
    let axis = [b.axis.x_axis, b.axis.y_axis, b.axis.z_axis];

    let n = side_sign * axis[axis_idx];
    let u = axis[(axis_idx + 1) % 3];
    let v = axis[(axis_idx + 2) % 3];

    let eu = b.extent[(axis_idx + 1) % 3];
    let ev = b.extent[(axis_idx + 2) % 3];

    let center = b.center + n * b.extent[axis_idx];
    let ueu = u * eu;
    let vev = v * ev;

    // CCW ordering
    [
        center + ueu + vev,
        center - ueu + vev,
        center - ueu - vev,
        center + ueu - vev,
    ]
}

/// Returns closest points between segment `P(s) = p1 + s * (p2 - p1)`,
/// `Q(t) = q1 + t * (q2 - q1)`
fn closest_edge_edge_points(p1: Vec3A, p2: Vec3A, q1: Vec3A, q2: Vec3A) -> (Vec3A, Vec3A) {
    let delta_p = p2 - p1;
    let delta_q = q2 - q1;

    let a = delta_p.length_squared();
    let e = delta_q.length_squared();

    if a <= f32::EPSILON && e <= f32::EPSILON {
        return (p1, q1);
    }

    let r = p1 - q1;
    let f = delta_q.dot(r);
    let (t, s) = if a <= f32::EPSILON {
        ((f / e).clamp(0.0, 1.0), 0.0)
    } else {
        let c = delta_p.dot(r);
        if e <= f32::EPSILON {
            (0.0, (-c / a).clamp(0.0, 1.0))
        } else {
            let b = delta_p.dot(delta_q);
            let denom = a * e - b * b;
            let s = if denom != 0.0 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };

            let t = (b * s + f) / e;
            if t < 0.0 {
                (0.0, (-c / a).clamp(0.0, 1.0))
            } else {
                (1.0, ((b - c) / a).clamp(0.0, 1.0))
            }
        }
    };

    (p1 + delta_p * s, q1 + delta_q * t)
}

/// Given a direction normal, find the OBB edge most aligned with that direction.
/// Returns the closest point on that edge.
fn closest_point_on_obb_as_edge(a: &Obb, dir: Vec3A) -> Vec3A {
    let mut best_axis = 0;
    let mut best = a.axis.x_axis.dot(dir).abs();
    for i in 1..3 {
        let val = a.axis.col(i).dot(dir).abs();
        if val > best {
            best = val;
            best_axis = i;
        }
    }

    let a_axis = a.axis.col(best_axis);
    let edge_dir = a_axis.cross(dir);
    if edge_dir.length_squared() < 1e-6 {
        // direction roughly parallel,
        // return center projection
        return a.center;
    }

    let extent = a_axis * a.extent[best_axis];
    let p1 = a.center - extent;
    let p2 = a.center + extent;

    closest_edge_edge_points(p1, p2, p1 + dir, p2 + dir).0
}

/// Given a direction normal, find the AABB edge most aligned with that direction.
/// Returns the closest point on that edge.
fn closest_point_on_aabb_as_edge(b: &Aabb, dir: Vec3A) -> Vec3A {
    const AXIS: [Vec3A; 3] = [Vec3A::X, Vec3A::Y, Vec3A::Z];
    let mut best_axis = 0;
    let mut best = AXIS[0].dot(dir).abs();
    for (i, axis) in AXIS[1..3].iter().enumerate() {
        let val = axis.dot(dir).abs();
        if val > best {
            best = val;
            best_axis = i;
        }
    }

    let min = b.center - b.extent;
    let max = b.center + b.extent;

    let mut pt = b.center.clamp(min, max);
    pt[best_axis] = if dir.dot(AXIS[best_axis]) >= 0.0 {
        max[best_axis]
    } else {
        min[best_axis]
    };

    pt
}

pub struct ObbObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_0_obj: CollisionObjectWrapper,
    compound_1_obj: CollisionObjectWrapper,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ObbObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_0_obj: CollisionObjectWrapper,
        compound_1_obj: CollisionObjectWrapper,
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
    fn process_collision<'a>(
        self,
        compound_0_obj: &'a CollisionObject,
        compound_1_obj: &'a CollisionObject,
    ) -> Option<PersistentManifold> {
        let Some(CollisionShapes::Compound(compound_0_ref)) = compound_0_obj.get_collision_shape()
        else {
            unreachable!();
        };

        let Some(CollisionShapes::Compound(compound_1_ref)) = compound_1_obj.get_collision_shape()
        else {
            unreachable!();
        };

        let org_0_trans = compound_0_obj.get_world_transform();
        let aabb_0 = compound_0_ref.get_aabb(org_0_trans);

        let org_1_trans = compound_1_obj.get_world_transform();
        let aabb_1 = compound_1_ref.get_aabb(org_1_trans);

        if !test_aabb_against_aabb(&aabb_0, &aabb_1) {
            return None;
        }

        // B
        let child_1 = compound_1_ref.child.as_ref().unwrap();
        let child_1_trans = child_1.transform;
        let new_child_1_world_trans = org_1_trans * child_1_trans;

        // A
        // translate A into B coords, B is now an AABB in comparison
        let child_0 = compound_0_ref.child.as_ref().unwrap();
        let child_0_from_local = new_child_1_world_trans.transpose() * child_0.transform;

        let obb = Obb {
            center: child_0_from_local.translation,
            axis: child_0_from_local.matrix3,
            extent: child_0.child_shape.get_half_extents(),
        };

        let aabb = Aabb {
            center: Vec3A::ZERO,
            extent: child_1.child_shape.get_half_extents(),
        };

        // solve for hit normal/depth
        let mut hit = collide_obb_aabb_sat(&obb, &aabb)?;
        let normal_on_b_in_world = new_child_1_world_trans.transform_vector3a(hit.normal);

        let mut manifold = PersistentManifold::new(
            self.compound_0_obj.object,
            self.compound_1_obj.object,
            false,
        );

        // solve for manifold points
        if hit.axis_type > 5 {
            // edge-edge fallback: approximate by 1 point along normal
            let pt_on_a = closest_point_on_obb_as_edge(&obb, hit.normal);
            let pt_on_b = closest_point_on_aabb_as_edge(&aabb, -hit.normal);
            let point = (pt_on_a + pt_on_b) * 0.5;

            manifold.add_contact_point(
                normal_on_b_in_world,
                new_child_1_world_trans.transform_point3a(point),
                hit.depth,
                -1,
                -1,
                self.contact_added_callback,
            );

            manifold.refresh_contact_points();
            return Some(manifold);
        }

        // Axis is 1 of the 6 face normals
        let obb_2 = aabb.into();

        let (ref_box, inc_box, ref_axis, ref_sign) = if hit.axis_type <= 2 {
            // A's face
            let ref_axis = usize::from(hit.axis_type);
            (
                &obb,
                &obb_2,
                ref_axis,
                hit.normal.dot(obb.axis.col(ref_axis)).signum(),
            )
        } else {
            // B's face
            let ref_axis = usize::from(hit.axis_type - 3);
            hit.normal *= -1.0;

            (
                &obb_2,
                &obb,
                ref_axis,
                hit.normal.dot(obb.axis.col(ref_axis)).signum(),
            )
        };

        let mut inc_axis = 0;
        let mut inc_sign = 0.0;

        // choose incident face on inc_box whose normal is most anti-parallel to the normal
        let mut best_dot = f32::INFINITY;
        for j in 0..3 {
            let axis = inc_box.axis.col(j);
            let d = axis.dot(hit.normal);

            let d_abs = d.abs();
            if d_abs < best_dot {
                best_dot = d_abs;
                inc_axis = j;
                inc_sign = d.signum();
            }
        }

        let inc_verts = get_face_verts(inc_box, inc_axis, inc_sign);

        let ref_face_n = ref_sign * ref_box.axis.col(ref_axis);
        let ref_u = ref_box.axis.col((ref_axis + 1) % 3);
        let ref_v = ref_box.axis.col((ref_axis + 2) % 3);

        let eu = ref_box.extent[(ref_axis + 1) % 3];
        let ev = ref_box.extent[(ref_axis + 2) % 3];

        let ref_center = ref_box.center + ref_face_n * ref_box.extent[ref_axis];
        let ref_plane_d = ref_face_n.dot(ref_center);

        // Clip incident face against reference face side planes
        let mut poly1 = inc_verts;
        let mut poly2 = [Vec3A::ZERO; 4];

        let mut n_pts = MANIFOLD_CACHE_SIZE;
        let mut clip = |n: Vec3A, d: f32| {
            let mut count = 0;

            for i in 0..n_pts {
                let a = poly1[i];
                let b = poly1[(i + 1) % n_pts];

                let da = d - n.dot(a);
                let db = d - n.dot(b);

                let ina = da.is_sign_positive();
                let inb = db.is_sign_positive();

                if !ina && !inb {
                    continue;
                }

                if ina && inb {
                    poly2[count] = b;
                } else if !inb {
                    let t = da / (da - db);
                    poly2[count] = a.lerp(b, t);
                } else {
                    let t = da / (da - db);
                    poly2[count] = a.lerp(b, t);
                    poly2[count + 1] = b;
                    count += 1;
                }

                count += 1;
            }

            n_pts = count;
            poly1 = poly2;
        };

        clip(ref_u, ref_u.dot(ref_center) + eu);
        clip(-ref_u, (-ref_u).dot(ref_center) + eu);
        clip(ref_v, ref_v.dot(ref_center) + ev);
        clip(-ref_v, (-ref_v).dot(ref_center) + ev);

        for p in poly1.into_iter().take(n_pts) {
            let dist = ref_face_n.dot(p) - ref_plane_d;
            let local_point = p - dist * ref_face_n;

            manifold.add_contact_point(
                normal_on_b_in_world,
                new_child_1_world_trans.transform_point3a(local_point),
                hit.depth,
                -1,
                -1,
                self.contact_added_callback,
            );
        }

        manifold.refresh_contact_points();
        Some(manifold)
    }
}
