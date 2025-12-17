use std::{cell::RefCell, rc::Rc};

use glam::Vec3A;

use super::{
    contact_solver_info::ContactSolverInfo, solver_body::SolverBody,
    solver_constraint::SolverConstraint,
};
use crate::bullet::{
    collision::{
        dispatch::collision_object::{CollisionObject, SpecialResolveInfo},
        narrowphase::{manifold_point::ContactPointFlags, persistent_manifold::PersistentManifold},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::{
        plane_space_1, plane_space_2,
        transform_util::{integrate_transform, integrate_transform_no_rot},
    },
};

pub struct SequentialImpulseConstraintSolver {
    pub tmp_solver_body_pool: Vec<SolverBody>,
    pub tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    pub tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    pub fixed_body_id: Option<usize>,
    pub least_squares_residual: f32,
}

impl Default for SequentialImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            fixed_body_id: None,
            least_squares_residual: 0.0,
        }
    }
}

impl SequentialImpulseConstraintSolver {
    fn restitution_curve(rel_vel: f32, restitution: f32, velocity_threshold: f32) -> f32 {
        if rel_vel.abs() < velocity_threshold {
            0.0
        } else {
            restitution * -rel_vel
        }
    }

    fn get_or_init_solver_body(
        &mut self,
        collision_objects: &[Rc<RefCell<RigidBody>>],
        body: &mut CollisionObject,
        time_step: f32,
    ) -> usize {
        if let Some(companion_id) = body.companion_id {
            return companion_id;
        }

        if !body.is_static_object() {
            let rb_ref = &collision_objects[body.get_world_array_index()];
            let rb = rb_ref.borrow();

            if rb.inverse_mass != 0.0 || body.is_kinematic_object() {
                let solver_body_id = self.tmp_solver_body_pool.len();
                body.companion_id = Some(solver_body_id);

                self.tmp_solver_body_pool
                    .push(SolverBody::new(&rb, time_step));
                return solver_body_id;
            }
        }

        if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            body.companion_id = Some(solver_body_id);
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        }
    }

    pub fn solve_group(
        &mut self,
        collision_objects: &[Rc<RefCell<RigidBody>>],
        non_static_bodies: &[usize],
        manifolds: &mut Vec<PersistentManifold>,
        info: &ContactSolverInfo,
    ) {
        self.solve_group_setup(collision_objects, non_static_bodies, manifolds, info);
        self.solve_group_iterations(info);
        self.solve_group_finish(collision_objects, info);
    }

    fn solve_group_setup(
        &mut self,
        collision_objects: &[Rc<RefCell<RigidBody>>],
        non_static_bodies: &[usize],
        manifolds: &mut Vec<PersistentManifold>,
        info: &ContactSolverInfo,
    ) {
        self.fixed_body_id = None;

        self.tmp_solver_body_pool.clear();
        self.tmp_solver_body_pool.reserve(manifolds.len() * 2);

        for manifold in manifolds.iter() {
            collision_objects[manifold.body0_idx]
                .borrow_mut()
                .collision_object
                .companion_id = None;
            collision_objects[manifold.body1_idx]
                .borrow_mut()
                .collision_object
                .companion_id = None;
        }

        for &rb_idx in non_static_bodies {
            let mut rb = collision_objects[rb_idx].borrow_mut();
            rb.collision_object.companion_id = None;

            if rb.inverse_mass != 0.0 || rb.collision_object.is_kinematic_object() {
                if !rb.collision_object.is_active() {
                    continue;
                }

                let solver_body_id = self.tmp_solver_body_pool.len();
                rb.collision_object.companion_id = Some(solver_body_id);

                self.tmp_solver_body_pool
                    .push(SolverBody::new(&rb, info.time_step));
            } else if self.fixed_body_id.is_none() {
                let solver_body_id = self.tmp_solver_body_pool.len();
                rb.collision_object.companion_id = Some(solver_body_id);

                self.fixed_body_id = Some(solver_body_id);
                self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            }
        }

        for manifold in manifolds.iter_mut() {
            let mut body0 = collision_objects[manifold.body0_idx].borrow_mut();
            let mut body1 = collision_objects[manifold.body1_idx].borrow_mut();

            let solver_body_id_a = self.get_or_init_solver_body(
                collision_objects,
                &mut body0.collision_object,
                info.time_step,
            );
            let solver_body_id_b = self.get_or_init_solver_body(
                collision_objects,
                &mut body1.collision_object,
                info.time_step,
            );

            debug_assert!(solver_body_id_a < self.tmp_solver_body_pool.len());
            debug_assert!(solver_body_id_b < self.tmp_solver_body_pool.len());
            debug_assert_ne!(solver_body_id_a, solver_body_id_b);
            let [solver_body_a, solver_body_b] = unsafe {
                self.tmp_solver_body_pool
                    .get_disjoint_unchecked_mut([solver_body_id_a, solver_body_id_b])
            };

            body0.collision_object.companion_id = Some(solver_body_id_a);
            body1.collision_object.companion_id = Some(solver_body_id_b);

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.position_world_on_a
                    - body0.collision_object.get_world_transform().translation;
                let rel_pos2 = cp.position_world_on_b
                    - body1.collision_object.get_world_transform().translation;

                if cp.is_special {
                    for (obj, rel_pos) in [
                        (&mut body0.collision_object, rel_pos1),
                        (&mut body1.collision_object, rel_pos2),
                    ] {
                        if !obj.is_static_object() {
                            obj.special_resolve_info.num_special_collisions += 1;
                            obj.special_resolve_info.friction = cp.combined_friction;
                            obj.special_resolve_info.restitution = cp.combined_restitution;
                            obj.special_resolve_info.total_normal += cp.normal_world_on_b;
                            obj.special_resolve_info.total_dist += rel_pos.length();
                        }
                    }
                }

                let rb0 = solver_body_a.original_body.map(|_| &body0);
                let rb1 = solver_body_b.original_body.map(|_| &body1);

                // setupContactConstraint
                let relaxation = info.sor;
                let inv_time_step = 1.0 / info.time_step;
                let erp = info.erp_2;

                let torque_axis_0 = rel_pos1.cross(cp.normal_world_on_b);
                let angular_component_a = rb0.map_or(Vec3A::ZERO, |rb| {
                    rb.inv_inertia_tensor_world.transpose() * torque_axis_0
                });

                let torque_axis_1 = rel_pos2.cross(cp.normal_world_on_b);
                let angular_component_b = rb1.map_or(Vec3A::ZERO, |rb| {
                    rb.inv_inertia_tensor_world.transpose() * -torque_axis_1
                });

                let denom0 = rb0.map_or(0.0, |rb| {
                    let vec = angular_component_a.cross(rel_pos1);
                    rb.inverse_mass + cp.normal_world_on_b.dot(vec)
                });
                let denom1 = rb1.map_or(0.0, |rb| {
                    let vec = angular_component_b.cross(rel_pos2);
                    rb.inverse_mass + cp.normal_world_on_b.dot(vec)
                });

                let jac_diag_ab_inv = relaxation / (denom0 + denom1);

                let (contact_normal_1, rel_pos1_cross_normal) = if rb0.is_some() {
                    (cp.normal_world_on_b, torque_axis_0)
                } else {
                    (Vec3A::ZERO, Vec3A::ZERO)
                };

                let (contact_normal_2, rel_pos2_cross_normal) = if rb1.is_some() {
                    (-cp.normal_world_on_b, -torque_axis_1)
                } else {
                    (Vec3A::ZERO, Vec3A::ZERO)
                };

                let penetration = cp.distance_1 + info.linear_slop;

                let vel1 = rb0.map_or(Vec3A::ZERO, |rb| rb.get_velocity_in_local_point(rel_pos1));
                let vel2 = rb1.map_or(Vec3A::ZERO, |rb| rb.get_velocity_in_local_point(rel_pos2));

                let vel = vel1 - vel2;
                let rel_vel = cp.normal_world_on_b.dot(vel);

                let restitution = Self::restitution_curve(
                    rel_vel,
                    cp.combined_restitution,
                    info.restitution_velocity_threshold,
                )
                .max(0.0);

                let (external_force_impulse_a, external_torque_impulse_a) = if rb0.is_some() {
                    (
                        solver_body_a.external_force_impulse,
                        solver_body_a.external_torque_impulse,
                    )
                } else {
                    (Vec3A::ZERO, Vec3A::ZERO)
                };

                let (external_force_impulse_b, external_torque_impulse_b) = if rb1.is_some() {
                    (
                        solver_body_b.external_force_impulse,
                        solver_body_b.external_torque_impulse,
                    )
                } else {
                    (Vec3A::ZERO, Vec3A::ZERO)
                };

                let vel_1_dot_n = contact_normal_1
                    .dot(solver_body_a.linear_velocity + external_force_impulse_a)
                    + rel_pos1_cross_normal
                        .dot(solver_body_a.angular_velocity + external_torque_impulse_a);

                let vel_2_dot_n = contact_normal_2
                    .dot(solver_body_b.linear_velocity + external_force_impulse_b)
                    + rel_pos2_cross_normal
                        .dot(solver_body_b.angular_velocity + external_torque_impulse_b);

                let rel_vel = vel_1_dot_n + vel_2_dot_n;

                let positional_error = if penetration > 0.0 {
                    0.0
                } else {
                    -penetration * erp * inv_time_step
                };

                let velocity_error = restitution - rel_vel;

                let penetration_impulse = positional_error * jac_diag_ab_inv;
                let velocity_impulse = velocity_error * jac_diag_ab_inv;

                debug_assert!(info.split_impulse);
                let (rhs, rhs_penetration) =
                    if penetration > info.split_impulse_penetration_threshold {
                        (penetration_impulse + velocity_impulse, 0.0)
                    } else {
                        (velocity_impulse, penetration_impulse)
                    };

                let applied_impulse = cp.applied_impulse * info.warmstarting_factor;
                if rb0.is_some() {
                    solver_body_a.internal_apply_impulse(
                        contact_normal_1 * solver_body_a.inv_mass,
                        angular_component_a,
                        applied_impulse,
                    );
                }

                if rb1.is_some() {
                    solver_body_b.internal_apply_impulse(
                        -contact_normal_2 * solver_body_b.inv_mass,
                        -angular_component_b,
                        -applied_impulse,
                    );
                }

                let friction_index = self.tmp_solver_contact_constraint_pool.len();
                self.tmp_solver_contact_constraint_pool
                    .push(SolverConstraint {
                        solver_body_id_a,
                        solver_body_id_b,
                        angular_component_a,
                        angular_component_b,
                        jac_diag_ab_inv,
                        contact_normal_1,
                        contact_normal_2,
                        rel_pos1_cross_normal,
                        rel_pos2_cross_normal,
                        rhs,
                        rhs_penetration,
                        applied_impulse,
                        friction_index: self.tmp_solver_contact_friction_constraint_pool.len(),
                        friction: cp.combined_friction,
                        is_special: cp.is_special,
                        // original_contact_point: Some(cp),
                        lower_limit: 0.0,
                        upper_limit: 1e10,
                        ..Default::default()
                    });

                // convertContactInner
                let vel1 = solver_body_a.get_velocity_in_local_point_no_delta(rel_pos1);
                let vel2 = solver_body_b.get_velocity_in_local_point_no_delta(rel_pos2);

                let vel = vel1 - vel2;
                let rel_vel = cp.normal_world_on_b.dot(vel);

                debug_assert!(
                    cp.contact_point_flags & ContactPointFlags::LateralFrictionInitialized as i32
                        == 0
                );

                cp.lateral_friction_dir_1 = vel - cp.normal_world_on_b * rel_vel;
                let lat_rel_vel = cp.lateral_friction_dir_1.length_squared();

                if lat_rel_vel > f32::EPSILON {
                    cp.lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
                } else {
                    (cp.lateral_friction_dir_1, cp.lateral_friction_dir_2) =
                        plane_space_2(cp.normal_world_on_b);
                }

                // addFrictionConstraint
                let normal_axis = cp.lateral_friction_dir_1;

                let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) =
                    rb0.map_or((Vec3A::ZERO, Vec3A::ZERO, Vec3A::ZERO), |rb| {
                        let torque_axis = rel_pos1.cross(normal_axis);

                        (
                            normal_axis,
                            torque_axis,
                            rb.inv_inertia_tensor_world.transpose() * torque_axis,
                        )
                    });

                let (contact_normal_2, rel_pos2_cross_normal, angular_component_b) =
                    rb1.map_or((Vec3A::ZERO, Vec3A::ZERO, Vec3A::ZERO), |rb| {
                        let normal_axis = -normal_axis;
                        let torque_axis = rel_pos2.cross(normal_axis);

                        (
                            normal_axis,
                            torque_axis,
                            rb.inv_inertia_tensor_world.transpose() * torque_axis,
                        )
                    });

                let denom0 = rb0.map_or(0.0, |rb| {
                    let vec = angular_component_a.cross(rel_pos1);
                    rb.inverse_mass + normal_axis.dot(vec)
                });
                let denom1 = rb1.map_or(0.0, |rb| {
                    let vec = angular_component_b.cross(rel_pos2);
                    rb.inverse_mass + normal_axis.dot(vec)
                });

                let jac_diag_ab_inv = relaxation / (denom0 + denom1);

                let vel_1_dot_n = contact_normal_1
                    .dot(solver_body_a.linear_velocity + external_force_impulse_a)
                    + rel_pos1_cross_normal.dot(solver_body_a.angular_velocity);

                let vel_2_dot_n = contact_normal_2
                    .dot(solver_body_b.linear_velocity + external_force_impulse_b)
                    + rel_pos2_cross_normal.dot(solver_body_b.angular_velocity);

                let rel_vel = vel_1_dot_n + vel_2_dot_n;

                let velocity_error = -rel_vel;
                let velocity_impulse = velocity_error * jac_diag_ab_inv;

                debug_assert!(
                    cp.contact_point_flags & ContactPointFlags::FrictionAnchor as i32 == 0
                );

                self.tmp_solver_contact_friction_constraint_pool
                    .push(SolverConstraint {
                        friction_index,
                        solver_body_id_a,
                        solver_body_id_b,
                        contact_normal_1,
                        contact_normal_2,
                        rel_pos1_cross_normal,
                        rel_pos2_cross_normal,
                        angular_component_a,
                        angular_component_b,
                        jac_diag_ab_inv,
                        rhs: velocity_impulse,
                        lower_limit: -cp.combined_friction,
                        upper_limit: cp.combined_friction,
                        friction: cp.combined_friction,
                        ..Default::default()
                    });
            }
        }

        manifolds.clear();

        for &body in non_static_bodies {
            let mut body = collision_objects[body].borrow_mut();
            if body
                .collision_object
                .special_resolve_info
                .num_special_collisions
                > 0
            {
                self.convert_contact_special(&body, info);
                body.collision_object.special_resolve_info = SpecialResolveInfo::DEFAULT;
            }
        }
    }

    fn convert_contact_special(&mut self, body: &RigidBody, info: &ContactSolverInfo) {
        let sri = &body.collision_object.special_resolve_info;
        let num_collisions = f32::from(sri.num_special_collisions);
        let distance = sri.total_dist / num_collisions;
        let normal_world_on_b = sri.total_normal / num_collisions;

        let friction_index = self.tmp_solver_contact_constraint_pool.len();

        let solver_body_id_a = body.collision_object.companion_id.unwrap();
        let solver_body_id_b = if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        };

        let solver_body_a = &mut self.tmp_solver_body_pool[solver_body_id_a];

        let rel_pos1 = normal_world_on_b * -distance;
        let relaxation = info.sor;

        let inv_time_step = 1.0 / info.time_step;
        let erp = info.erp_2;

        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        let angular_component_a = body.inv_inertia_tensor_world.transpose() * torque_axis_0;

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inverse_mass + normal_world_on_b.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let (contact_normal_1, rel_pos1_cross_normal) = (normal_world_on_b, torque_axis_0);

        let penetration = distance + info.linear_slop;

        let vel = body.get_velocity_in_local_point(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let restitution = Self::restitution_curve(
            rel_vel,
            sri.restitution,
            info.restitution_velocity_threshold,
        )
        .max(0.0);

        let (external_force_impulse_a, external_torque_impulse_a) = (
            solver_body_a.external_force_impulse,
            solver_body_a.external_torque_impulse,
        );

        let rel_vel = contact_normal_1
            .dot(solver_body_a.linear_velocity + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.angular_velocity + external_torque_impulse_a);

        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let velocity_error = restitution - rel_vel;

        let penetration_impulse = positional_error * jac_diag_ab_inv;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;

        debug_assert!(info.split_impulse);
        let (rhs, rhs_penetration) = if penetration > info.split_impulse_penetration_threshold {
            (penetration_impulse + velocity_impulse, 0.0)
        } else {
            (velocity_impulse, penetration_impulse)
        };

        self.tmp_solver_contact_constraint_pool
            .push(SolverConstraint {
                solver_body_id_a,
                solver_body_id_b,
                angular_component_a,
                jac_diag_ab_inv,
                contact_normal_1,
                rel_pos1_cross_normal,
                rhs,
                rhs_penetration,
                friction: sri.friction,
                lower_limit: 0.0,
                upper_limit: 1e10,
                ..Default::default()
            });

        let vel = solver_body_a.get_velocity_in_local_point_no_delta(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let mut lateral_friction_dir_1 = vel - normal_world_on_b * rel_vel;
        let lat_rel_vel = lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir_1 = plane_space_1(normal_world_on_b);
        }

        // addFrictionConstraint
        let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) = {
            let torque_axis = rel_pos1.cross(lateral_friction_dir_1);

            (
                lateral_friction_dir_1,
                torque_axis,
                body.inv_inertia_tensor_world * torque_axis,
            )
        };

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inverse_mass + lateral_friction_dir_1.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let rel_vel = contact_normal_1
            .dot(solver_body_a.linear_velocity + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.angular_velocity + external_torque_impulse_a);

        let velocity_error = -rel_vel;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;

        self.tmp_solver_contact_friction_constraint_pool
            .push(SolverConstraint {
                friction_index,
                solver_body_id_a,
                solver_body_id_b,
                contact_normal_1,
                rel_pos1_cross_normal,
                angular_component_a,
                jac_diag_ab_inv,
                rhs: velocity_impulse,
                lower_limit: -sri.friction,
                upper_limit: sri.friction,
                friction: sri.friction,
                ..Default::default()
            });
    }

    fn solve_group_split_impulse_iterations(&mut self, info: &ContactSolverInfo) {
        debug_assert!(info.split_impulse);

        let mut should_run = (1u64 << self.tmp_solver_contact_constraint_pool.len()) - 1;

        for _ in 0..info.num_iterations {
            for (i, contact) in self
                .tmp_solver_contact_constraint_pool
                .iter_mut()
                .enumerate()
            {
                let mask = 1 << i;
                if should_run & mask == 0 {
                    continue;
                }

                debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
                let [body_a, body_b] = unsafe {
                    self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                        contact.solver_body_id_a,
                        contact.solver_body_id_b,
                    ])
                };

                let residual = contact.resolve_split_penetration_impulse(body_a, body_b);
                // println!("residual: {residual:?}");
                if residual * residual == 0.0 {
                    should_run ^= mask;
                }
            }

            if should_run == 0 {
                break;
            }
        }
    }

    fn solve_single_iteration(&mut self) -> f32 {
        let mut least_squares_residual = 0.0;

        for contact in &mut self.tmp_solver_contact_constraint_pool {
            if contact.is_special {
                continue;
            }

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_lower_limit(body_a, body_b);
            // println!("residual: {residual:?}");
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        for contact in &mut self.tmp_solver_contact_friction_constraint_pool {
            let total_impulse =
                self.tmp_solver_contact_constraint_pool[contact.friction_index].applied_impulse;
            if total_impulse <= 0.0 {
                continue;
            }

            let limit = contact.friction * total_impulse;
            contact.lower_limit = -limit;
            contact.upper_limit = limit;

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_generic(body_a, body_b);
            // println!("residual: {residual:?}");
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }

    fn solve_group_iterations(&mut self, info: &ContactSolverInfo) {
        self.solve_group_split_impulse_iterations(info);

        for _ in 0..info.num_iterations {
            self.least_squares_residual = self.solve_single_iteration();
            // println!("least_squares_residual: {:?}", self.least_squares_residual);
            if self.least_squares_residual == 0.0 {
                break;
            }
        }
    }

    fn solve_group_finish(
        &mut self,
        collision_objects: &[Rc<RefCell<RigidBody>>],
        info: &ContactSolverInfo,
    ) {
        // writeBackBodies
        for solver in &mut self.tmp_solver_body_pool {
            let Some(mut body) = solver
                .original_body
                .map(|idx| collision_objects[idx].borrow_mut())
            else {
                continue;
            };

            solver.linear_velocity += solver.delta_linear_velocity;
            solver.angular_velocity += solver.delta_angular_velocity;

            debug_assert!(info.split_impulse);
            if solver.push_velocity.length_squared() != 0.0
                || solver.turn_velocity.length_squared() != 0.0
            {
                if body.collision_object.no_rot {
                    integrate_transform_no_rot(
                        &mut solver.world_transform,
                        solver.push_velocity,
                        info.time_step,
                    );
                } else {
                    integrate_transform(
                        &mut solver.world_transform,
                        solver.push_velocity,
                        solver.turn_velocity * info.split_impulse_turn_erp,
                        info.time_step,
                    );
                }
            }

            body.set_linear_velocity(solver.linear_velocity + solver.external_force_impulse);
            body.set_angular_velocity(solver.angular_velocity + solver.external_torque_impulse);

            body.collision_object
                .set_world_transform(solver.world_transform);
        }

        self.tmp_solver_body_pool.clear();
        self.tmp_solver_contact_constraint_pool.clear();
        self.tmp_solver_contact_friction_constraint_pool.clear();
    }
}
