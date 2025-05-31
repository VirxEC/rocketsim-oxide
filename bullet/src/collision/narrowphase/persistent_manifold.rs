use super::manifold_point::ManifoldPoint;
use crate::{
    UserInfoTypes,
    collision::dispatch::{
        collision_object::{CollisionFlags, CollisionObject},
        internal_edge_utility::adjust_internal_edge_contacts,
    },
    linear_math::{AffineTranspose, plane_space},
};
use arrayvec::ArrayVec;
use glam::{Vec3A, Vec4};
use std::{cell::RefCell, mem, rc::Rc};

pub const CONTACT_BREAKING_THRESHOLD: f32 = 0.02;
pub const MANIFOLD_CACHE_SIZE: usize = 4;

enum ContactManifoldTypes {
    MinContactManifoldType = 1024,
    PersistentManifoldType,
}

pub struct PersistentManifold {
    object_type: i32,
    pub point_cache: ArrayVec<ManifoldPoint, MANIFOLD_CACHE_SIZE>,
    pub body0: Rc<RefCell<CollisionObject>>,
    pub body1: Rc<RefCell<CollisionObject>>,
    pub contact_breaking_threshold: f32,
    pub contact_processing_threshold: f32,
    pub companion_id_a: i32,
    pub companion_id_b: i32,
    index_1a: i32,
    pub is_swapped: bool,
}

impl PersistentManifold {
    pub fn new(
        body0: Rc<RefCell<CollisionObject>>,
        body1: Rc<RefCell<CollisionObject>>,
        is_swapped: bool,
    ) -> Self {
        let body0_cbt = body0
            .borrow()
            .get_collision_shape()
            .unwrap()
            .borrow()
            .get_contact_breaking_threshold(CONTACT_BREAKING_THRESHOLD);
        let body1_cbt = body1
            .borrow()
            .get_collision_shape()
            .unwrap()
            .borrow()
            .get_contact_breaking_threshold(CONTACT_BREAKING_THRESHOLD);
        let contact_breaking_threshold = body0_cbt.min(body1_cbt);
        let contact_processing_threshold = body0
            .borrow()
            .contact_processing_threshold
            .min(body1.borrow().contact_processing_threshold);

        Self {
            body0,
            body1,
            contact_breaking_threshold,
            contact_processing_threshold,
            object_type: ContactManifoldTypes::PersistentManifoldType as i32,
            point_cache: const { ArrayVec::new_const() },
            companion_id_a: 0,
            companion_id_b: 0,
            index_1a: 0,
            is_swapped,
        }
    }

    fn calculate_combined_friction(body0: &CollisionObject, body1: &CollisionObject) -> f32 {
        if body0.is_static_object() || body1.is_static_object() {
            body0.friction.min(body1.friction)
        } else {
            body0.friction * body1.friction
        }
    }

    fn calculate_combined_restitution(body0: &CollisionObject, body1: &CollisionObject) -> f32 {
        if body0.is_static_object() || body1.is_static_object() {
            body0.restitution.max(body1.restitution)
        } else {
            body0.restitution * body1.restitution
        }
    }

    // fn calculate_combined_rolling_friction(
    //     body0: &CollisionObject,
    //     body1: &CollisionObject,
    // ) -> f32 {
    //     const MAX_FRICTION: f32 = 10.0;

    //     let friction =
    //         body0.rolling_friction * body1.friction + body0.friction * body1.rolling_friction;
    //     friction.clamp(-MAX_FRICTION, MAX_FRICTION)
    // }

    // fn calculate_combined_spinning_friction(
    //     body0: &CollisionObject,
    //     body1: &CollisionObject,
    // ) -> f32 {
    //     const MAX_FRICTION: f32 = 10.0;

    //     let friction =
    //         body0.spinning_friction * body1.friction + body0.friction * body1.spinning_friction;
    //     friction.clamp(-MAX_FRICTION, MAX_FRICTION)
    // }

    #[inline]
    fn get_res(new_contact_local: Vec3A, point1: Vec3A, point2: Vec3A, point3: Vec3A) -> f32 {
        (new_contact_local - point1)
            .cross(point2 - point3)
            .length_squared()
    }

    #[inline]
    fn get_res_0(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[1].local_point_a,
            self.point_cache[3].local_point_a,
            self.point_cache[2].local_point_a,
        )
    }

    #[inline]
    fn get_res_1(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[0].local_point_a,
            self.point_cache[3].local_point_a,
            self.point_cache[2].local_point_a,
        )
    }

    #[inline]
    fn get_res_2(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[0].local_point_a,
            self.point_cache[3].local_point_a,
            self.point_cache[1].local_point_a,
        )
    }

    #[inline]
    fn get_res_3(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[0].local_point_a,
            self.point_cache[2].local_point_a,
            self.point_cache[1].local_point_a,
        )
    }

    fn sort_cached_points(&self, new_contact: &ManifoldPoint) -> usize {
        let mut max_penetration_index = MANIFOLD_CACHE_SIZE;
        let mut max_penetration = new_contact.distance_1;
        for (i, contact) in self.point_cache.iter().enumerate() {
            if contact.distance_1 < max_penetration {
                max_penetration_index = i;
                max_penetration = contact.distance_1;
            }
        }

        let res = match max_penetration_index {
            0 => Vec4::new(
                0.,
                self.get_res_1(new_contact.local_point_a),
                self.get_res_2(new_contact.local_point_a),
                self.get_res_3(new_contact.local_point_a),
            ),
            1 => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                0.,
                self.get_res_2(new_contact.local_point_a),
                self.get_res_3(new_contact.local_point_a),
            ),
            2 => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                self.get_res_1(new_contact.local_point_a),
                0.,
                self.get_res_3(new_contact.local_point_a),
            ),
            3 => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                self.get_res_1(new_contact.local_point_a),
                self.get_res_2(new_contact.local_point_a),
                0.,
            ),
            _ => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                self.get_res_1(new_contact.local_point_a),
                self.get_res_2(new_contact.local_point_a),
                self.get_res_3(new_contact.local_point_a),
            ),
        };

        res.max_position()
    }

    fn add_manifold_point(&mut self, contact: ManifoldPoint) -> usize {
        let num_points = self.point_cache.len();
        if num_points == MANIFOLD_CACHE_SIZE {
            let index = self.sort_cached_points(&contact);
            self.point_cache[index] = contact;
            index
        } else {
            self.point_cache.push(contact);
            num_points
        }
    }

    pub fn add_contact_point(
        &mut self,
        normal_on_b_in_world: Vec3A,
        point_in_world: Vec3A,
        depth: f32,
        part_id_0: i32,
        part_id_1: i32,
        index_0: i32,
        index_1: i32,
    ) {
        if depth > self.contact_breaking_threshold {
            return;
        }

        let body0 = self.body0.borrow();
        let body1 = self.body1.borrow();

        let point_a = point_in_world + normal_on_b_in_world * depth;
        let (local_a, local_b) = (
            body0.get_world_transform().inv_x_form(point_a),
            body1.get_world_transform().inv_x_form(point_in_world),
        );

        let mut new_pt = ManifoldPoint::new(local_a, local_b, normal_on_b_in_world, depth);
        new_pt.position_world_on_a = point_a;
        new_pt.position_world_on_b = point_in_world;

        new_pt.combined_friction = Self::calculate_combined_friction(&body0, &body1);
        new_pt.combined_restitution = Self::calculate_combined_restitution(&body0, &body1);

        // debug_assert_eq!(body0.rolling_friction, 0.0);
        // debug_assert_eq!(body1.rolling_friction, 0.0);
        // new_pt.combined_rolling_friction =
        //     Self::calculate_combined_rolling_friction(&body0, &body1);

        // debug_assert_eq!(body0.spinning_friction, 0.0);
        // debug_assert_eq!(body1.spinning_friction, 0.0);
        // new_pt.combined_spinning_friction =
        //     Self::calculate_combined_spinning_friction(&body0, &body1);

        debug_assert_eq!(
            body0.collision_flags & CollisionFlags::HasContactStiffnessDamping as i32,
            0
        );
        debug_assert_eq!(
            body1.collision_flags & CollisionFlags::HasContactStiffnessDamping as i32,
            0
        );

        debug_assert_eq!(
            body0.collision_flags & CollisionFlags::HasFrictionAnchor as i32,
            0
        );
        debug_assert_eq!(
            body1.collision_flags & CollisionFlags::HasFrictionAnchor as i32,
            0
        );

        (new_pt.lateral_friction_dir_1, new_pt.lateral_friction_dir_2) =
            plane_space(new_pt.normal_world_on_b);

        if self.is_swapped {
            new_pt.part_id_0 = part_id_1;
            new_pt.part_id_1 = part_id_0;
            new_pt.index_0 = index_1;
            new_pt.index_1 = index_0;
        } else {
            new_pt.part_id_0 = part_id_0;
            new_pt.part_id_1 = part_id_1;
            new_pt.index_0 = index_0;
            new_pt.index_1 = index_1;
        }

        drop(body0);
        drop(body1);

        let insert_index = self.add_manifold_point(new_pt);

        let (body0, body1) = if self.is_swapped {
            (self.body1.borrow(), self.body0.borrow())
        } else {
            (self.body0.borrow(), self.body1.borrow())
        };

        Self::bullet_contact_added_callback(&mut self.point_cache[insert_index], &body0, &body1);
    }

    fn bullet_contact_added_callback<'a>(
        contact_point: &mut ManifoldPoint,
        mut body_a: &'a CollisionObject,
        mut body_b: &'a CollisionObject,
    ) {
        debug_assert!(body_a.has_contact_response() || body_b.has_contact_response());

        let should_swap = if body_a.user_index != -1 && body_b.user_index != -1 {
            body_a.user_index > body_b.user_index
        } else {
            body_b.user_index != -1
        };

        if should_swap {
            mem::swap(&mut body_a, &mut body_b);
        }

        let user_index_a = body_a.user_index;
        let user_index_b = body_b.user_index;

        if user_index_a == UserInfoTypes::Car as i32 {
            todo!()
        } else if user_index_a == UserInfoTypes::Ball as i32 {
            if user_index_b == UserInfoTypes::DropshotTile as i32 {
                todo!()
            } else if user_index_b == -1 {
                contact_point.is_special = true;
            }
        }

        adjust_internal_edge_contacts(contact_point, body_a, body_b);
    }

    fn remove_contact_point(&mut self, _idx: usize) {
        todo!()
    }

    pub fn refresh_contact_points(&mut self) {
        if self.point_cache.is_empty() {
            return;
        }

        let body0 = self.body0.borrow();
        let body1 = self.body1.borrow();

        let tr_a = body0.get_world_transform();
        let tr_b = body1.get_world_transform();

        for manifold_point in &mut self.point_cache {
            manifold_point.position_world_on_a =
                tr_a.transform_point3a(manifold_point.local_point_a);
            manifold_point.position_world_on_b =
                tr_b.transform_point3a(manifold_point.local_point_b);
            manifold_point.distance_1 = (manifold_point.position_world_on_a
                - manifold_point.position_world_on_b)
                .dot(manifold_point.normal_world_on_b);
            manifold_point.life_time += 1;
        }

        drop(body0);
        drop(body1);

        let contact_breaking_threshold_sq =
            self.contact_breaking_threshold * self.contact_breaking_threshold;

        for i in (0..self.point_cache.len()).rev() {
            let point = &self.point_cache[i];
            if point.distance_1 > self.contact_breaking_threshold {
                self.remove_contact_point(i);
            } else {
                let projected_point =
                    point.position_world_on_a - point.position_world_on_b * point.distance_1;
                let projected_difference = point.position_world_on_b - projected_point;
                let distance_2d = projected_difference.dot(projected_difference);
                if distance_2d > contact_breaking_threshold_sq {
                    self.remove_contact_point(i);
                }
            }
        }
    }
}
