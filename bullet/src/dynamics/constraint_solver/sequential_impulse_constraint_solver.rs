use super::{
    contact_solver_info::ContactSolverInfo, solver_body::SolverBody,
    solver_constraint::SolverConstraint,
};
use crate::{
    collision::{
        dispatch::collision_object::{CollisionObject, SpecialResolveInfo},
        narrowphase::{
            manifold_point::ContactPointFlags,
            persistent_manifold::{MANIFOLD_CACHE_SIZE, PersistentManifold},
        },
    },
    dynamics::{constraint_solver::contact_solver_info::SolverMode, rigid_body::RigidBody},
    linear_math::{
        plane_space,
        transform_util::{integrate_transform, integrate_transform_no_rot},
    },
};
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

// pub type SingleConstraintRowSolver = fn(&mut SolverBody, &mut SolverBody, &SolverConstraint) -> f32;

pub struct SequentialImpulseConstraintSolver {
    pub tmp_solver_body_pool: Vec<SolverBody>,
    pub tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    // pub tmp_solver_non_contact_constraint_pool: Vec<SolverConstraint>,
    pub tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    // pub tmp_solver_contact_rolling_constraint_pool: Vec<SolverConstraint>,
    // pub order_tmp_constraint_pool: Vec<i32>,
    // pub order_non_contact_constraint_pool: Vec<i32>,
    // pub order_friction_constraint_pool: Vec<i32>,
    pub max_override_num_solver_iterations: u32,
    pub fixed_body_id: Option<usize>,
    pub kinematic_body_unique_id_to_solver_body_table: Vec<i32>,
    // pub resolve_single_constraint_row_generic: SingleConstraintRowSolver,
    // pub resolve_single_constraint_row_lower_limit: SingleConstraintRowSolver,
    // pub resolve_split_penetration_impulse: SingleConstraintRowSolver,
    pub cached_solver_mode: i32,
    pub least_squares_residual: f32,
    pub bt_seed_2: u64,
    // btSolverAnalyticsData m_analyticsData;
}

impl Default for SequentialImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            // tmp_solver_non_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            // tmp_solver_contact_rolling_constraint_pool: Vec::new(),
            // order_tmp_constraint_pool: Vec::new(),
            // order_non_contact_constraint_pool: Vec::new(),
            // order_friction_constraint_pool: Vec::new(),
            max_override_num_solver_iterations: 0,
            fixed_body_id: None,
            kinematic_body_unique_id_to_solver_body_table: Vec::new(),
            // resolve_single_constraint_row_generic: Self::resolve_single_constraint_row_generic,
            // resolve_single_constraint_row_lower_limit:
            //     Self::resolve_single_constraint_row_lower_limit,
            // resolve_split_penetration_impulse: Self::resolve_split_penetration_impulse,
            cached_solver_mode: 0,
            least_squares_residual: 0.0,
            bt_seed_2: 0,
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
        bodies: &[Rc<RefCell<RigidBody>>],
        body: &Rc<RefCell<CollisionObject>>,
        time_step: f32,
    ) -> usize {
        if let Some(companion_id) = body.borrow().companion_id {
            return companion_id;
        }

        if let Some(rb) = bodies
            .iter()
            .find(|rb| Rc::ptr_eq(&rb.borrow().collision_object, body))
        {
            let rb_ref = rb.borrow();
            if rb_ref.inverse_mass != 0.0 || rb_ref.collision_object.borrow().is_kinematic_object()
            {
                let solver_body_id = self.tmp_solver_body_pool.len();
                body.borrow_mut().companion_id = Some(solver_body_id);

                self.tmp_solver_body_pool
                    .push(SolverBody::new(rb.clone(), time_step));
                return solver_body_id;
            }
        }

        if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            body.borrow_mut().companion_id = Some(solver_body_id);
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        }
    }

    pub fn solve_group(
        &mut self,
        bodies: &[Rc<RefCell<RigidBody>>],
        manifolds: &mut Vec<PersistentManifold>,
        info: &ContactSolverInfo,
    ) {
        self.solve_group_setup(bodies, manifolds, info);
        self.solve_group_iterations(info);
        self.solve_group_finish(info);
    }

    fn solve_group_setup(
        &mut self,
        bodies: &[Rc<RefCell<RigidBody>>],
        manifolds: &mut Vec<PersistentManifold>,
        info: &ContactSolverInfo,
    ) {
        self.fixed_body_id = None;
        self.max_override_num_solver_iterations = 0;

        self.tmp_solver_body_pool.clear();
        self.tmp_solver_body_pool.reserve(manifolds.len() * 2);

        for manifold in manifolds.drain(..) {
            let solver_body_id_a =
                self.get_or_init_solver_body(bodies, &manifold.body0, info.time_step);
            let solver_body_id_b =
                self.get_or_init_solver_body(bodies, &manifold.body1, info.time_step);

            debug_assert_ne!(solver_body_id_a, solver_body_id_b);
            let [solver_body_a, solver_body_b] = unsafe {
                self.tmp_solver_body_pool
                    .get_disjoint_unchecked_mut([solver_body_id_a, solver_body_id_b])
            };

            manifold.body0.borrow_mut().companion_id = Some(solver_body_id_a);
            manifold.body1.borrow_mut().companion_id = Some(solver_body_id_b);

            let trans0 = manifold.body0.borrow().get_world_transform().translation;
            let trans1 = manifold.body1.borrow().get_world_transform().translation;

            for mut cp in manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.position_world_on_a - trans0;
                let rel_pos2 = cp.position_world_on_b - trans1;

                if cp.is_special {
                    for (mut obj, rel_pos) in [
                        (manifold.body0.borrow_mut(), rel_pos1),
                        (manifold.body1.borrow_mut(), rel_pos2),
                    ] {
                        obj.special_resolve_info.num_special_collisions += 1;
                        obj.special_resolve_info.friction = cp.combined_friction;
                        obj.special_resolve_info.restitution = cp.combined_restitution;
                        obj.special_resolve_info.total_normal += cp.normal_world_on_b;
                        obj.special_resolve_info.total_dist += rel_pos.length();
                    }
                }

                let rb0 = solver_body_a.original_body.as_ref();
                let rb1 = solver_body_b.original_body.as_ref();

                // setupContractConstraint
                let relaxation = info.sor;
                let inv_time_step = 1.0 / info.time_step;
                debug_assert_eq!(info.global_cfm, 0.0);
                let erp = info.erp_2;

                let torque_axis_0 = rel_pos1.cross(cp.normal_world_on_b);
                let angular_component_a = if let Some(rb) = rb0 {
                    rb.borrow().inv_inertia_tensor_world
                        * torque_axis_0
                        * rb.borrow().angular_factor
                } else {
                    Vec3A::ZERO
                };

                let torque_axis_1 = rel_pos2.cross(cp.normal_world_on_b);
                let angular_component_b = if let Some(rb) = rb1 {
                    rb.borrow().inv_inertia_tensor_world
                        * -torque_axis_0
                        * rb.borrow().angular_factor
                } else {
                    Vec3A::ZERO
                };

                let denom0 = if let Some(rb) = rb0 {
                    let vec = angular_component_a.cross(rel_pos1);
                    rb.borrow().inverse_mass + cp.normal_world_on_b.dot(vec)
                } else {
                    0.0
                };

                let denom1 = if let Some(rb) = rb1 {
                    let vec = angular_component_b.cross(rel_pos2);
                    rb.borrow().inverse_mass + cp.normal_world_on_b.dot(vec)
                } else {
                    0.0
                };

                let jac_diag_ab_inv = relaxation / (denom0 + denom1);

                let (contact_normal_1, rel_pos1_cross_normal) = if rb0.is_some() {
                    (cp.normal_world_on_b, torque_axis_0)
                } else {
                    (Vec3A::ZERO, Vec3A::ZERO)
                };

                let (contact_normal_2, rel_pos2_cross_normal) = if rb0.is_some() {
                    (-cp.normal_world_on_b, -torque_axis_1)
                } else {
                    (Vec3A::ZERO, Vec3A::ZERO)
                };

                let penetration = cp.distance_1 + info.linear_slop;

                let vel1 = if let Some(rb) = rb0 {
                    rb.borrow().get_velocity_in_local_point(rel_pos1)
                } else {
                    Vec3A::ZERO
                };

                let vel2 = if let Some(rb) = rb1 {
                    rb.borrow().get_velocity_in_local_point(rel_pos2)
                } else {
                    Vec3A::ZERO
                };

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

                let (rhs, rhs_penetration) = if !info.split_impulse
                    || penetration > info.split_impulse_penetration_threshold
                {
                    (penetration_impulse + velocity_impulse, 0.0)
                } else {
                    (velocity_impulse, penetration_impulse)
                };

                debug_assert!(info.solver_mode & SolverMode::UseWarmstarting as i32 != 0);
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
                // if cp.combined_rolling_friction > 0.0 {
                //     unimplemented!()
                // }

                let vel1 = solver_body_a.get_velocity_in_local_point_no_delta(rel_pos1);
                let vel2 = solver_body_b.get_velocity_in_local_point_no_delta(rel_pos2);

                let vel = vel1 - vel2;
                let rel_vel = cp.normal_world_on_b.dot(vel);

                debug_assert!(
                    info.solver_mode & SolverMode::EnableFrictionDirectionCaching as i32 == 0
                        || cp.contact_point_flags
                            & ContactPointFlags::LateralFrictionInitialized as i32
                            == 0
                );

                cp.lateral_friction_dir_1 = vel - cp.normal_world_on_b * rel_vel;
                let lat_rel_vel = cp.lateral_friction_dir_1.length_squared();

                debug_assert!(
                    info.solver_mode & SolverMode::DisableVelocityDependentFrictionDirection as i32
                        == 0
                );

                if lat_rel_vel > f32::EPSILON {
                    cp.lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
                    debug_assert!(!manifold.body0.borrow().has_anisotropic_friction);
                    debug_assert!(!manifold.body1.borrow().has_anisotropic_friction);
                } else {
                    (cp.lateral_friction_dir_1, cp.lateral_friction_dir_2) =
                        plane_space(cp.normal_world_on_b);
                    debug_assert!(!manifold.body0.borrow().has_anisotropic_friction);
                    debug_assert!(!manifold.body1.borrow().has_anisotropic_friction);
                }

                debug_assert!(info.solver_mode & SolverMode::Use2FrictionDirections as i32 == 0);

                let rb0 = solver_body_a.original_body.as_ref();
                let rb1 = solver_body_b.original_body.as_ref();

                // addFrictionConstraint
                let normal_axis = cp.lateral_friction_dir_1;

                let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) =
                    if let Some(rb) = rb1 {
                        let rb = rb.borrow();
                        let torque_axis = rel_pos1.cross(normal_axis);

                        (
                            normal_axis,
                            torque_axis,
                            rb.inv_inertia_tensor_world * torque_axis * rb.angular_factor,
                        )
                    } else {
                        (Vec3A::ZERO, Vec3A::ZERO, Vec3A::ZERO)
                    };

                let (contact_normal_2, rel_pos2_cross_normal, angular_component_b) =
                    if let Some(rb) = rb0 {
                        let rb = rb.borrow();
                        let torque_axis = rel_pos1.cross(normal_axis);

                        (
                            -normal_axis,
                            torque_axis,
                            rb.inv_inertia_tensor_world * torque_axis * rb.angular_factor,
                        )
                    } else {
                        (Vec3A::ZERO, Vec3A::ZERO, Vec3A::ZERO)
                    };

                let denom0 = if let Some(rb) = rb0 {
                    let vec = angular_component_a.cross(rel_pos1);
                    rb.borrow().inverse_mass + normal_axis.dot(vec)
                } else {
                    0.0
                };

                let denom1 = if let Some(rb) = rb1 {
                    let vec = angular_component_b.cross(rel_pos2);
                    rb.borrow().inverse_mass + normal_axis.dot(vec)
                } else {
                    0.0
                };

                let jac_diag_ab_inv = relaxation / (denom0 + denom1);

                let vel_1_dot_n = contact_normal_1
                    .dot(solver_body_a.linear_velocity + external_force_impulse_a)
                    + rel_pos1_cross_normal
                        .dot(solver_body_a.angular_velocity + external_torque_impulse_a);

                let vel_2_dot_n = contact_normal_2
                    .dot(solver_body_b.linear_velocity + external_force_impulse_b)
                    + rel_pos2_cross_normal
                        .dot(solver_body_b.angular_velocity + external_torque_impulse_b);

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

        for body in bodies {
            let body = body.borrow();
            if body
                .collision_object
                .borrow()
                .special_resolve_info
                .num_special_collisions
                > 0
            {
                self.convert_contact_special(&body.collision_object.borrow(), info);
                body.collision_object.borrow_mut().special_resolve_info =
                    SpecialResolveInfo::DEFAULT;
            }
        }
    }

    fn convert_contact_special(&mut self, obj: &CollisionObject, info: &ContactSolverInfo) {
        let sri = &obj.special_resolve_info;

        let num_collisions = f32::from(sri.num_special_collisions);
        let distance = sri.total_dist / num_collisions;
        let normal_world_on_b = sri.total_normal / num_collisions;

        // combined_friction: sri.friction,
        // combined_restitution: sri.restitution,

        let friction_index = self.tmp_solver_contact_constraint_pool.len();

        let solver_body_id_a = obj.companion_id.unwrap();
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

        let rb0 = solver_body_a.original_body.as_ref();

        let inv_time_step = 1.0 / info.time_step;
        debug_assert_eq!(info.global_cfm, 0.0);
        let erp = info.erp_2;

        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        let angular_component_a = if let Some(rb) = rb0 {
            rb.borrow().inv_inertia_tensor_world * torque_axis_0 * rb.borrow().angular_factor
        } else {
            Vec3A::ZERO
        };

        let denom = if let Some(rb) = rb0 {
            let vec = angular_component_a.cross(rel_pos1);
            rb.borrow().inverse_mass + normal_world_on_b.dot(vec)
        } else {
            0.0
        };

        let jac_diag_ab_inv = relaxation / denom;

        let (contact_normal_1, rel_pos1_cross_normal) = if rb0.is_some() {
            (normal_world_on_b, torque_axis_0)
        } else {
            (Vec3A::ZERO, Vec3A::ZERO)
        };

        let penetration = distance + info.linear_slop;

        let vel = if let Some(rb) = rb0 {
            rb.borrow().get_velocity_in_local_point(rel_pos1)
        } else {
            Vec3A::ZERO
        };

        let rel_vel = normal_world_on_b.dot(vel);

        let restitution = Self::restitution_curve(
            rel_vel,
            sri.restitution,
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

        let (rhs, rhs_penetration) =
            if !info.split_impulse || penetration > info.split_impulse_penetration_threshold {
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

        debug_assert!(
            info.solver_mode & SolverMode::DisableVelocityDependentFrictionDirection as i32 == 0
        );

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            (lateral_friction_dir_1, _) = plane_space(normal_world_on_b);
        }

        debug_assert!(info.solver_mode & SolverMode::Use2FrictionDirections as i32 == 0);

        let rb0 = solver_body_a.original_body.as_ref();

        // addFrictionConstraint
        let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) = if let Some(rb) = rb0 {
            let rb = rb.borrow();
            let torque_axis = rel_pos1.cross(lateral_friction_dir_1);

            (
                lateral_friction_dir_1,
                torque_axis,
                rb.inv_inertia_tensor_world * torque_axis * rb.angular_factor,
            )
        } else {
            (Vec3A::ZERO, Vec3A::ZERO, Vec3A::ZERO)
        };

        let denom = if let Some(rb) = rb0 {
            let vec = angular_component_a.cross(rel_pos1);
            rb.borrow().inverse_mass + lateral_friction_dir_1.dot(vec)
        } else {
            0.0
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
        debug_assert_eq!(MANIFOLD_CACHE_SIZE, 4);

        let mut should_run = 0xfu8;

        for i in self.tmp_solver_contact_constraint_pool.len()..MANIFOLD_CACHE_SIZE {
            should_run ^= 1 << i;
        }

        for _ in 0..info.num_iterations {
            for (i, contact) in self.tmp_solver_contact_constraint_pool.iter().enumerate() {
                let mask = 1 << i;
                // println!("{should_run:b} & (1 << {i}) = {}", should_run & mask);
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

                if residual * residual <= f32::EPSILON {
                    should_run ^= mask;
                }
            }

            if should_run == 0 {
                break;
            }
        }
    }

    fn solve_single_iteration(&mut self, info: &ContactSolverInfo) -> f32 {
        let mut least_squares_residual = 0.0;

        debug_assert_eq!(info.solver_mode & SolverMode::RandomizeOrder as i32, 0);
        debug_assert_eq!(
            info.solver_mode & SolverMode::InterleaveContactAndFrictionConstraints as i32,
            0
        );

        for contact in &mut self.tmp_solver_contact_constraint_pool {
            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_lower_limit(body_a, body_b);
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
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }

    fn solve_group_iterations(&mut self, info: &ContactSolverInfo) {
        self.solve_group_split_impulse_iterations(info);

        let max_iterations = if self.max_override_num_solver_iterations > info.num_iterations {
            self.max_override_num_solver_iterations
        } else {
            info.num_iterations
        };

        for _ in 0..max_iterations {
            self.least_squares_residual = self.solve_single_iteration(info);
            if self.least_squares_residual <= f32::EPSILON {
                break;
            }
        }
    }

    fn solve_group_finish(&mut self, info: &ContactSolverInfo) {
        // writeBackBodies
        for mut solver in self.tmp_solver_body_pool.drain(..) {
            let Some(mut body) = solver.original_body.as_ref().map(|body| body.borrow_mut()) else {
                continue;
            };

            solver.linear_velocity += solver.delta_linear_velocity;
            solver.angular_velocity += solver.delta_angular_velocity;

            if solver.push_velocity.length_squared() != 0.0
                || solver.turn_velocity.length_squared() != 0.0
            {
                solver.world_transform = if body.collision_object.borrow().no_rot {
                    integrate_transform_no_rot(
                        &solver.world_transform,
                        solver.push_velocity,
                        info.time_step,
                    )
                } else {
                    integrate_transform(
                        &solver.world_transform,
                        solver.push_velocity,
                        solver.turn_velocity,
                        info.time_step,
                    )
                };
            }

            body.set_linear_velocity(solver.linear_velocity + solver.external_force_impulse);
            body.set_angular_velocity(solver.angular_velocity + solver.external_torque_impulse);

            debug_assert!(info.split_impulse);
            let mut co = body.collision_object.borrow_mut();
            co.set_world_transform(solver.world_transform);
            co.companion_id = None;
        }

        self.tmp_solver_contact_constraint_pool.clear();
        self.tmp_solver_contact_friction_constraint_pool.clear();
    }
}
