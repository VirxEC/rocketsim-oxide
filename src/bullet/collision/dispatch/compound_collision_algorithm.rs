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

fn project_triangle(tri: &TriangleShape, obb_center: Vec3A, axis: Vec3A) -> (f32, f32) {
    let projections = Vec3A::new(
        (tri.points[0] - obb_center).dot(axis),
        (tri.points[1] - obb_center).dot(axis),
        (tri.points[2] - obb_center).dot(axis),
    );

    (projections.min_element(), projections.max_element())
}

fn closest_point_on_segment(p: Vec3A, a: Vec3A, b: Vec3A) -> Vec3A {
    let ab = b - a;
    let t = (p - a).dot(ab) / ab.dot(ab);
    a + ab * t.clamp(0.0, 1.0)
}

/// Check SAT between OBB and triangle; if collision, return penetration depth & normal
fn obb_triangle_sat(obb: &Obb, tri: &TriangleShape) -> Option<Hit> {
    // Prepare candidate axes
    let axes = [
        tri.normal,
        obb.axis.x_axis,
        obb.axis.y_axis,
        obb.axis.z_axis,
    ];

    let mut min_overlap = f32::INFINITY;
    let mut min_axis = Vec3A::ZERO;
    let mut min_axis_index = 0;

    for (axis_index, axis) in axes.into_iter().enumerate() {
        // Project OBB
        let r_obb = obb.project_obb_radius(axis);

        // Project triangle
        let (tri_min, tri_max) = project_triangle(tri, obb.center, axis);

        // Compute projected interval of triangle relative to 0 (centered at obb.center): [tri_min, tri_max]
        // Compute overlap: OBB projection is [-r_obb, +r_obb]
        // So overlap amount = (r_obb) - distance from center to triangle interval center, but simpler:
        // overlap = r_obb + (r_obb) - (tri_max - tri_min)? No: better to think:
        // If projections are [-r_obb, +r_obb] and [tri_min, tri_max], the distance between centers is (tri_min + tri_max)/2,
        // But simpler: compute overlap length = (r_obb * 2) + (tri_max - tri_min) - separation
        // However, Eberly’s method is simpler: check if interval [tri_min, tri_max] intersects [-r_obb, +r_obb];
        // Then compute overlap as min( r_obb - tri_min, tri_max + r_obb ) (depending on which side is penetrating).
        // We'll compute separation:
        let tri_center = (tri_min + tri_max) * 0.5;
        let tri_radius = (tri_max - tri_min) * 0.5;
        let distance = tri_center.abs(); // because OBB interval is symmetric at 0
        let overlap = (r_obb - distance) + tri_radius;

        if overlap < 0.0 {
            // found separating axis
            return None;
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            min_axis = axis;
            min_axis_index = axis_index;
        }
    }

    // Edge-edge cross axes: edges of triangle × axes of OBB
    let mut axis_index = 3;
    for &tri_edge in &tri.edges {
        for obb_axis in &[obb.axis.x_axis, obb.axis.y_axis, obb.axis.z_axis] {
            axis_index += 1;
            let Some(axis) = obb_axis.cross(tri_edge).try_normalize() else {
                // Parallel edges — skip this axis
                continue;
            };

            let r_obb = obb.project_obb_radius(axis);
            let (tri_min, tri_max) = project_triangle(tri, obb.center, axis);

            let tri_center = (tri_min + tri_max) * 0.5;
            let tri_radius = (tri_max - tri_min) * 0.5;
            let distance = tri_center.abs(); // because OBB interval is symmetric at 0
            let overlap = (r_obb - distance) + tri_radius;

            if overlap < 0.0 {
                return None;
            }

            if overlap < min_overlap {
                min_overlap = overlap;
                min_axis = axis;
                min_axis_index = axis_index;
            }
        }
    }

    // No separating axis found => collision
    // Choose the axis of least penetration as the collision normal
    let mut normal = min_axis;

    // Relative vector from OBB center to triangle (pick one triangle point, e.g., points[0])
    let d0 = tri.points[0] - obb.center;
    // Make sure normal points from OBB → triangle (based on d0)
    if d0.dot(normal) < 0.0 {
        normal = -normal;
    }

    Some(Hit {
        axis_index: min_axis_index,
        depth: min_overlap,
        normal,
    })
}

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: CollisionObjectWrapper<'a>,
    pub tri_obj: &'a CollisionObject,
    pub aabb: &'a Aabb,
    pub box_shape: &'a BoxShape,
    pub contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> TriangleCallback for ConvexTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb: &Aabb,
        triangle_index: usize,
    ) -> bool {
        if !test_aabb_against_aabb(tri_aabb, self.aabb) {
            return true;
        }

        let obb = Obb::new(
            self.convex_obj.world_transform.translation,
            self.convex_obj.world_transform.matrix3,
            self.box_shape.get_half_extents(),
        );

        let Some(hit) = obb_triangle_sat(&obb, triangle) else {
            return true;
        };

        let normal_on_b_in_world = self
            .convex_obj
            .world_transform
            .transform_vector3a(hit.normal);

        match hit.axis_index {
            0 => {
                // Triangle normal axis
                // Find closest point on OBB to triangle plane
                let plane_d = -triangle.normal.dot(triangle.points[0]);
                let dist_to_plane = triangle.normal.dot(obb.center) + plane_d;
                let closest_point = obb.center - triangle.normal * dist_to_plane;

                self.manifold.add_contact_point(
                    self.convex_obj.object,
                    self.tri_obj,
                    normal_on_b_in_world,
                    closest_point,
                    dist_to_plane,
                    -1,
                    triangle_index as i32,
                    self.contact_added_callback,
                );
            }
            1..4 => {
                // OBB face axis — use face projection
                let face_axis_idx = hit.axis_index - 1;
                let face_verts = obb.get_face_verts(face_axis_idx, 1.0);

                // Find closest point on triangle to face
                let mut closest_point = face_verts[0];
                let mut min_dist_sq = f32::INFINITY;
                for &face_vert in &face_verts {
                    // Project face_vert onto triangle plane
                    let plane_d = -triangle.normal.dot(triangle.points[0]);
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
                let aabb = box_shape.get_aabb(&convex_in_triangle_space);

                let mut convex_triangle_callback = {
                    ConvexTriangleCallback {
                        manifold: PersistentManifold::new(
                            self.compound_obj,
                            self.other_obj,
                            self.is_swapped,
                        ),
                        convex_obj: compound_obj_wrap,
                        tri_obj: self.other_obj,
                        aabb: &aabb,
                        box_shape,
                        contact_added_callback: self.contact_added_callback,
                    }
                };

                tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);

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
