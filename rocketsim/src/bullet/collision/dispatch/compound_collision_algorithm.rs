use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
            convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            box_shape::BoxShape, collision_shape::CollisionShapes,
            triangle_callback::TriangleCallback, triangle_shape::TriangleShape,
        },
    },
    linear_math::{
        AffineExt,
        aabb_util_2::{Aabb, test_aabb_against_aabb},
        obb::Obb,
    },
};

struct Hit {
    depth: f32,
    normal: Vec3A,
    axis_index: usize,
}

fn project_triangle(tri: &TriangleShape, axis: Vec3A) -> (f32, f32) {
    let projections = Vec3A::new(
        tri.points[0].dot(axis),
        tri.points[1].dot(axis),
        tri.points[2].dot(axis),
    );

    (projections.min_element(), projections.max_element())
}

fn closest_point_on_segment(p: Vec3A, a: Vec3A, b: Vec3A) -> Vec3A {
    let ab = b - a;
    let t = (p - a).dot(ab) / ab.dot(ab);
    a + ab * t.clamp(0.0, 1.0)
}

/// Project an AABB onto an axis, returning the “radius” (half-projection length)
pub fn project_aabb_radius(extent: Vec3A, axis: Vec3A) -> f32 {
    axis.abs().dot(extent)
}

/// Check SAT between AABB and triangle; if collision, return penetration depth & normal
fn aabb_triangle_sat(extent: Vec3A, tri: &TriangleShape, tri_normal_overlap: f32) -> Option<Hit> {
    const IDENT_AXES: [Vec3A; 3] = [Vec3A::X, Vec3A::Y, Vec3A::Z];

    let mut min_overlap = tri_normal_overlap;
    let mut min_axis = tri.normal;
    let mut min_axis_index = 0;

    let mut axis_index = 0;
    for obb_axis in IDENT_AXES {
        axis_index += 1;

        let r_obb = project_aabb_radius(extent, obb_axis);
        let (tri_min, tri_max) = project_triangle(tri, obb_axis);

        let tri_center = (tri_min + tri_max) * 0.5;
        let tri_radius = (tri_max - tri_min) * 0.5;
        let distance = tri_center.abs();
        let overlap = (distance - r_obb) - tri_radius;

        if overlap > 0.0 {
            // found separating axis
            return None;
        }

        if overlap > min_overlap {
            min_overlap = overlap;
            min_axis = obb_axis;
            min_axis_index = axis_index;
        }
    }

    // Edge-edge cross axes: edges of triangle × axes of OBB
    for &tri_edge in &tri.edges {
        for obb_axis in IDENT_AXES {
            axis_index += 1;
            let Some(axis) = obb_axis.cross(tri_edge).try_normalize() else {
                // Parallel edges — skip this axis
                continue;
            };

            let r_obb = project_aabb_radius(extent, axis);
            let (tri_min, tri_max) = project_triangle(tri, axis);

            let tri_center = (tri_min + tri_max) * 0.5;
            let tri_radius = (tri_max - tri_min) * 0.5;
            let distance = tri_center.abs();
            let overlap = (distance - r_obb) - tri_radius;

            if overlap > 0.0 {
                return None;
            }

            if overlap > min_overlap {
                min_overlap = overlap;
                min_axis = axis;
                min_axis_index = axis_index;
            }
        }
    }

    // No separating axis found => collision
    Some(Hit {
        axis_index: min_axis_index,
        depth: min_overlap,
        normal: min_axis,
    })
}

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: CollisionObjectWrapper<'a>,
    pub tri_obj: &'a CollisionObject,
    pub local_convex_aabb: &'a Aabb,
    pub box_shape: &'a BoxShape,
    pub contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    fn get_triangle_separation(&self, triangle: &[Vec3A; 3], inv_tri_normal: Vec3A) -> f32 {
        let local_pt = self.box_shape.local_get_supporting_vertex(inv_tri_normal);
        let proj_dist_pt = inv_tri_normal.dot(local_pt);
        let proj_dist_tr = inv_tri_normal.dot(triangle[0]);

        proj_dist_tr - proj_dist_pt
    }
}

impl<T: ContactAddedCallback> TriangleCallback for ConvexTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb: &Aabb,
        triangle_index: usize,
    ) -> bool {
        if !test_aabb_against_aabb(tri_aabb, self.local_convex_aabb) {
            return true;
        }

        // transform the triangle into OBB space
        let xform1 = self.convex_obj.world_transform.transpose();
        let xform2 = self.tri_obj.get_world_transform();
        let triangle_in_obb = Affine3A {
            matrix3: xform1.matrix3 * xform2.matrix3,
            translation: xform1.transform_point3a(xform2.translation),
        };

        let triangle_points_in_obb = [
            triangle_in_obb.transform_point3a(triangle.points[0]),
            triangle_in_obb.transform_point3a(triangle.points[1]),
            triangle_in_obb.transform_point3a(triangle.points[2]),
        ];

        let local_triangle = TriangleShape::new(triangle_points_in_obb);

        // check if this is fully on one side of the triangle
        // check the back side first because we expect that to be the most likely
        let back_dist =
            self.get_triangle_separation(&local_triangle.points, -local_triangle.normal);
        if back_dist > self.manifold.contact_breaking_threshold {
            return true;
        }

        // now check the other side
        let front_dist =
            self.get_triangle_separation(&local_triangle.points, local_triangle.normal);
        if front_dist > self.manifold.contact_breaking_threshold {
            return true;
        }

        let tri_normal_overlap = front_dist.min(back_dist);
        if tri_normal_overlap > 0.0 {
            return true;
        }

        let obb = Obb::new(
            self.convex_obj.world_transform.translation,
            self.convex_obj.world_transform.matrix3,
            self.box_shape.get_half_extents(),
        );

        let Some(hit) = aabb_triangle_sat(obb.extent, &local_triangle, tri_normal_overlap) else {
            return true;
        };

        // transform hit.normal back into world space from obb space
        let normal_on_b_in_world = self
            .convex_obj
            .world_transform
            .transform_vector3a(hit.normal);

        match hit.axis_index {
            0 => {
                // Triangle normal axis
                // Find closest point on OBB to triangle plane
                let dist_to_plane = -local_triangle.normal.dot(local_triangle.points[0]);
                let closest_point = obb.center - triangle.normal * dist_to_plane;

                self.manifold.add_contact_point(
                    self.convex_obj.object,
                    self.tri_obj,
                    normal_on_b_in_world,
                    closest_point,
                    hit.depth,
                    -1,
                    triangle_index as i32,
                    self.contact_added_callback,
                );
            }
            1..4 => {
                // OBB face axis — use face projection
                let face_axis_idx = hit.axis_index - 1;
                let face_verts = obb.get_face_verts(face_axis_idx, 1.0);
                let plane_d = -triangle.normal.dot(triangle.points[0]);

                // Find closest point on triangle to face
                let mut closest_point = Vec3A::ZERO;
                let mut min_dist_sq = f32::INFINITY;
                for &face_vert in &face_verts {
                    // Project face_vert onto triangle plane
                    let dist_to_plane = triangle.normal.dot(face_vert) + plane_d;
                    let proj_point = face_vert - triangle.normal * dist_to_plane;

                    let dist_sq = (proj_point - face_vert).length_squared();
                    if dist_sq < min_dist_sq {
                        min_dist_sq = dist_sq;
                        closest_point = proj_point;
                    }
                }

                self.manifold.add_contact_point(
                    self.convex_obj.object,
                    self.tri_obj,
                    normal_on_b_in_world,
                    closest_point,
                    hit.depth,
                    -1,
                    triangle_index as i32,
                    self.contact_added_callback,
                );
            }
            axis_index => {
                // Edge-edge axis — approximate contact point as midpoint between closest points
                // on edges
                let (tri_edge_idx, obb_axis_idx) = ((axis_index - 4) / 3, (axis_index - 4) % 3);
                let obb_axis = obb.axis.col(obb_axis_idx);

                // Get two vertices of triangle edge
                let tri_a = triangle.points[tri_edge_idx];
                let tri_b = triangle.points[(tri_edge_idx + 1) % 3];

                // Get two points on OBB edge along obb_axis
                let obb_extent = obb.extent[obb_axis_idx];
                let obb_a = obb.center + obb_axis * obb_extent;
                let obb_b = obb.center - obb_axis * obb_extent;

                let pt_on_tri = closest_point_on_segment(obb.center, tri_a, tri_b);
                let pt_on_obb = closest_point_on_segment(obb.center, obb_a, obb_b);
                let contact_point = (pt_on_tri + pt_on_obb) * 0.5;

                self.manifold.add_contact_point(
                    self.convex_obj.object,
                    self.tri_obj,
                    normal_on_b_in_world,
                    contact_point,
                    hit.depth,
                    -1,
                    triangle_index as i32,
                    self.contact_added_callback,
                );
            }
        }

        true
    }
}

struct CompoundLeafCallback<'a, T: ContactAddedCallback> {
    compound_obj: &'a CollisionObject,
    other_obj: &'a CollisionObject,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> CompoundLeafCallback<'a, T> {
    pub const fn new(
        compound_obj: &'a CollisionObject,
        other_obj: &'a CollisionObject,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_obj,
            other_obj,
            is_swapped,
            contact_added_callback,
        }
    }

    pub fn process_child_shape(&mut self) -> Option<PersistentManifold> {
        let Some(CollisionShapes::Compound(compound_shape)) =
            self.compound_obj.get_collision_shape()
        else {
            unreachable!()
        };

        let org_trans = *self.compound_obj.get_world_transform();

        let child = compound_shape.child.as_ref().unwrap();
        let child_trans = child.transform;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &child.child_shape;
        let aabb1 = box_shape.get_aabb(&new_child_world_trans);

        let other_col_shape = self.other_obj.get_collision_shape().unwrap();
        let aabb2 = other_col_shape.get_aabb(self.other_obj.get_world_transform());

        if !test_aabb_against_aabb(&aabb1, &aabb2) {
            return None;
        }

        let compound_obj_wrap = CollisionObjectWrapper {
            object: self.compound_obj,
            world_transform: new_child_world_trans,
        };

        match other_col_shape {
            CollisionShapes::TriangleMesh(tri_mesh) => {
                let xform1 = self.other_obj.get_world_transform().transpose();
                let xform2 = new_child_world_trans;
                let convex_in_triangle_space = Affine3A {
                    matrix3: xform1.matrix3 * xform2.matrix3,
                    translation: xform1.transform_point3a(xform2.translation),
                };
                let aabb_in_triangle = box_shape.get_aabb(&convex_in_triangle_space);

                let mut convex_triangle_callback = {
                    ConvexTriangleCallback {
                        manifold: PersistentManifold::new(
                            self.compound_obj,
                            self.other_obj,
                            self.is_swapped,
                        ),
                        convex_obj: compound_obj_wrap,
                        tri_obj: self.other_obj,
                        local_convex_aabb: &aabb_in_triangle,
                        box_shape,
                        contact_added_callback: self.contact_added_callback,
                    }
                };

                tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb_in_triangle);

                convex_triangle_callback
                    .manifold
                    .refresh_contact_points(self.compound_obj, self.other_obj);
                if convex_triangle_callback.manifold.point_cache.is_empty() {
                    None
                } else {
                    Some(convex_triangle_callback.manifold)
                }
            }
            CollisionShapes::StaticPlane(_) => ConvexPlaneCollisionAlgorithm::new(
                compound_obj_wrap,
                self.other_obj,
                self.is_swapped,
                self.contact_added_callback,
            )
            .process_collision(),
            _ => todo!(),
        }
    }
}

pub struct CompoundCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_obj: &'a CollisionObject,
    other_obj: &'a CollisionObject,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> CompoundCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_obj: &'a CollisionObject,
        other_obj: &'a CollisionObject,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_obj,
            other_obj,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for CompoundCollisionAlgorithm<'_, T> {
    fn process_collision(self) -> Option<PersistentManifold> {
        let mut compound_leaf_callback = CompoundLeafCallback::new(
            self.compound_obj,
            self.other_obj,
            self.is_swapped,
            self.contact_added_callback,
        );

        compound_leaf_callback
            .process_child_shape()
            .and_then(|manifold| (!manifold.point_cache.is_empty()).then_some(manifold))
    }
}
