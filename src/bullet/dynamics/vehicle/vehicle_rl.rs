use super::{
    raycaster::VehicleRaycaster,
    wheel_info::{WheelInfo, WheelInfoConstructionInfo},
};
use crate::{
    bullet::dynamics::{
        constraint_solver::contact_constraint::{
            resolve_single_bilateral, resolve_single_collision,
        },
        discrete_dynamics_world::DiscreteDynamicsWorld,
        rigid_body::RigidBody,
    },
    consts::btvehicle::SUSPENSION_SUBTRACTION,
};
use arrayvec::ArrayVec;
use glam::{Affine3A, Mat3A, Quat, Vec3A};
use std::{cell::RefCell, rc::Rc};

pub struct WheelInfoRL {
    pub wheel_info: WheelInfo,
    pub is_in_contact_with_world: bool,
    pub steer_angle: f32,
    pub vel_at_contact_point: Vec3A,
    pub lat_friction: f32,
    pub long_friction: f32,
    pub impulse: Vec3A,
    pub suspsension_force_scale: f32,
    pub extra_pushback: f32,
}

impl WheelInfoRL {
    pub fn new(ci: WheelInfoConstructionInfo) -> Self {
        Self {
            wheel_info: WheelInfo::new(ci),
            is_in_contact_with_world: false,
            steer_angle: 0.0,
            vel_at_contact_point: Vec3A::ZERO,
            lat_friction: 0.0,
            long_friction: 0.0,
            impulse: Vec3A::ZERO,
            suspsension_force_scale: 0.0,
            extra_pushback: 0.0,
        }
    }

    fn update_wheel_transform_ws(&mut self, chassis_trans: &Affine3A) {
        self.is_in_contact_with_world = false;
        self.wheel_info.raycast_info.is_in_contact = false;
        self.wheel_info.raycast_info.hard_point_ws =
            chassis_trans.transform_point3a(self.wheel_info.chassis_connection_point_cs);
        self.wheel_info.raycast_info.wheel_direction_ws =
            chassis_trans.matrix3 * self.wheel_info.wheel_direction_cs;
        self.wheel_info.raycast_info.wheel_axle_ws =
            chassis_trans.matrix3 * self.wheel_info.wheel_axle_cs;
    }
}

pub struct VehicleTuning {
    pub suspension_stiffness: f32,
    pub suspension_compression: f32,
    pub suspension_damping: f32,
    pub max_suspension_travel_cm: f32,
    pub friction_slip: f32,
    pub max_suspension_force: f32,
}

impl Default for VehicleTuning {
    fn default() -> Self {
        Self {
            suspension_stiffness: 5.88,
            suspension_compression: 0.83,
            suspension_damping: 0.88,
            max_suspension_travel_cm: 500.0,
            friction_slip: 10.5,
            max_suspension_force: 6000.0,
        }
    }
}

pub struct VehicleRL {
    forward_ws: Vec<Vec3A>,
    axle: Vec<Vec3A>,
    forward_impulse: Vec<f32>,
    side_impulse: Vec<f32>,
    raycaster: VehicleRaycaster,
    pitch_control: f32,
    steering_value: f32,
    chassis_body: Rc<RefCell<RigidBody>>,
    index_right_axis: usize,
    index_up_axis: usize,
    index_forward_axis: usize,
    pub wheel_info: ArrayVec<WheelInfoRL, 4>,
}

impl VehicleRL {
    pub fn new(chassis_body: Rc<RefCell<RigidBody>>, raycaster: VehicleRaycaster) -> Self {
        Self {
            raycaster,
            chassis_body,
            forward_ws: Vec::new(),
            axle: Vec::new(),
            forward_impulse: Vec::new(),
            side_impulse: Vec::new(),
            pitch_control: 0.0,
            steering_value: 0.0,
            index_right_axis: 1,
            index_up_axis: 2,
            index_forward_axis: 0,
            wheel_info: ArrayVec::new(),
        }
    }

    pub fn get_chassis_world_transform(&self) -> Affine3A {
        *self
            .chassis_body
            .borrow()
            .collision_object
            .borrow()
            .get_world_transform()
    }

    pub fn get_up_vector(&self) -> Vec3A {
        self.chassis_body
            .borrow()
            .collision_object
            .borrow()
            .get_world_transform()
            .matrix3
            .col(self.index_up_axis)
    }

    pub fn get_forward_vector(&self) -> Vec3A {
        self.chassis_body
            .borrow()
            .collision_object
            .borrow()
            .get_world_transform()
            .matrix3
            .col(self.index_forward_axis)
    }

    pub fn get_forward_speed(&self) -> f32 {
        self.chassis_body
            .borrow()
            .linear_velocity
            .dot(self.get_forward_vector())
    }

    pub fn get_upwards_dir_from_wheel_contacts(&self) -> Vec3A {
        let mut sum_contact_dir = Vec3A::ZERO;
        for wheel in &self.wheel_info {
            if wheel.wheel_info.raycast_info.is_in_contact {
                sum_contact_dir += wheel.wheel_info.raycast_info.contact_normal_ws;
            }
        }

        if sum_contact_dir == Vec3A::ZERO {
            self.get_up_vector()
        } else {
            sum_contact_dir.normalize_or_zero()
        }
    }

    fn update_wheel_transform(&mut self, wheel_idx: usize) {
        let wheel = &mut self.wheel_info[wheel_idx];
        wheel.update_wheel_transform_ws(
            self.chassis_body
                .borrow()
                .collision_object
                .borrow()
                .get_world_transform(),
        );
        let up = -wheel.wheel_info.raycast_info.wheel_direction_ws;
        let right = wheel.wheel_info.raycast_info.wheel_axle_ws;
        let fwd = up.cross(right).normalize();

        let steering_orn = Quat::from_axis_angle(up.into(), wheel.steer_angle);
        let steering_mat = Mat3A::from_quat(steering_orn);

        let mut basis2 = Mat3A::ZERO;
        *basis2.col_mut(self.index_right_axis) = -right;
        *basis2.col_mut(self.index_up_axis) = up;
        *basis2.col_mut(self.index_forward_axis) = fwd;

        wheel.wheel_info.world_transform = Affine3A {
            matrix3: steering_mat * basis2,
            translation: wheel.wheel_info.raycast_info.hard_point_ws
                + wheel.wheel_info.wheel_direction_cs
                    * wheel.wheel_info.raycast_info.suspension_length,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add_wheel(
        &mut self,
        connection_point_cs: Vec3A,
        wheel_direction_cs0: Vec3A,
        wheel_axle_cs: Vec3A,
        suspension_rest_length: f32,
        wheel_radius: f32,
        tuning: &VehicleTuning,
        is_front_wheel: bool,
    ) {
        let ci = WheelInfoConstructionInfo {
            chassis_connection_cs: connection_point_cs,
            wheel_direction_cs: wheel_direction_cs0,
            wheel_axle_cs,
            suspension_rest_length,
            wheel_radius,
            suspension_stiffness: tuning.suspension_stiffness,
            wheels_damping_compression: tuning.suspension_compression,
            wheels_damping_relaxation: tuning.suspension_damping,
            friction_slip: tuning.friction_slip,
            is_front_wheel,
            max_suspension_travel_cm: tuning.max_suspension_travel_cm,
            max_suspension_force: tuning.max_suspension_force,
        };

        let wheel_idx = self.wheel_info.len();
        self.wheel_info.push(WheelInfoRL::new(ci));
        self.update_wheel_transform(wheel_idx);
    }

    pub const fn get_num_wheels(&self) -> usize {
        self.wheel_info.len()
    }

    fn ray_cast(&mut self, collision_world: &DiscreteDynamicsWorld, wheel_idx: usize) -> f32 {
        let up = self.get_up_vector();
        let num_wheels = self.get_num_wheels();
        let wheel = &mut self.wheel_info[wheel_idx];
        wheel.update_wheel_transform_ws(
            self.chassis_body
                .borrow()
                .collision_object
                .borrow()
                .get_world_transform(),
        );

        let mut depth = -1.0;

        let suspension_travel = wheel.wheel_info.max_suspension_travel_cm / 100.0;
        let real_ray_length = wheel.wheel_info.suspension_rest_length_1
            + suspension_travel
            + wheel.wheel_info.wheels_radius
            - SUSPENSION_SUBTRACTION;

        let source = wheel.wheel_info.raycast_info.hard_point_ws;
        let target = source + (wheel.wheel_info.raycast_info.wheel_direction_ws * real_ray_length);
        wheel.wheel_info.raycast_info.contact_point_ws = target;
        wheel.wheel_info.raycast_info.ground_object = None;

        let Some(ray_results) = self.raycaster.cast_ray(
            collision_world,
            source,
            target,
            &self.chassis_body.borrow().collision_object,
        ) else {
            wheel.wheel_info.raycast_info.suspension_length =
                wheel.wheel_info.suspension_rest_length_1 + suspension_travel;
            wheel.wheel_info.suspension_relative_velcity = 0.0;
            wheel.wheel_info.raycast_info.contact_normal_ws =
                -wheel.wheel_info.raycast_info.wheel_direction_ws;
            wheel.wheel_info.clipped_inv_contact_dot_suspension = 1.0;
            wheel.extra_pushback = 0.0;

            return depth;
        };

        let rb = ray_results.rigid_body.borrow();
        let co = rb.collision_object.borrow();
        wheel.wheel_info.raycast_info.contact_point_ws = ray_results.hit_point_in_world;
        depth = real_ray_length * ray_results.dist_fraction;
        wheel.wheel_info.raycast_info.contact_normal_ws = ray_results.hit_normal_in_world;
        wheel.wheel_info.raycast_info.is_in_contact = true;
        wheel.is_in_contact_with_world = co.is_static_object();

        wheel.wheel_info.raycast_info.ground_object = Some(ray_results.rigid_body.clone());

        let wheel_trace_len_sq = (wheel.wheel_info.raycast_info.hard_point_ws
            - wheel.wheel_info.raycast_info.contact_point_ws)
            .dot(up);
        wheel.wheel_info.raycast_info.suspension_length =
            wheel_trace_len_sq - wheel.wheel_info.wheels_radius;

        let min_suspension_len = wheel.wheel_info.suspension_rest_length_1 - suspension_travel;
        let max_suspension_len = wheel.wheel_info.suspension_rest_length_1 + suspension_travel;
        wheel.wheel_info.raycast_info.suspension_length = wheel
            .wheel_info
            .raycast_info
            .suspension_length
            .clamp(min_suspension_len, max_suspension_len);

        let cb = self.chassis_body.borrow();
        let rel_pos = wheel.wheel_info.raycast_info.contact_point_ws
            - cb.collision_object
                .borrow()
                .get_world_transform()
                .translation;
        wheel.vel_at_contact_point = cb.get_velocity_in_local_point(rel_pos);

        let proj_vel = wheel
            .wheel_info
            .raycast_info
            .contact_normal_ws
            .dot(wheel.vel_at_contact_point);
        let denom = wheel.wheel_info.raycast_info.contact_normal_ws.dot(up);

        if denom > 0.1 {
            let inv = 1.0 / denom;
            wheel.wheel_info.suspension_relative_velcity = proj_vel * inv;
            wheel.wheel_info.clipped_inv_contact_dot_suspension = inv;
        } else {
            wheel.wheel_info.suspension_relative_velcity = 0.0;
            wheel.wheel_info.clipped_inv_contact_dot_suspension = 10.0;
        }

        if wheel.is_in_contact_with_world {
            let ray_pushback_thresh = wheel.wheel_info.suspension_rest_length_1
                + wheel.wheel_info.wheels_radius
                - SUSPENSION_SUBTRACTION;
            if wheel_trace_len_sq < ray_pushback_thresh {
                let wheel_trace_dist_delta = wheel_trace_len_sq - ray_pushback_thresh;
                let collision_result = resolve_single_collision(
                    &cb,
                    &rb,
                    ray_results.hit_point_in_world,
                    ray_results.hit_normal_in_world,
                    &collision_world.dynamics_world.solver_info,
                    wheel_trace_dist_delta,
                );

                wheel.extra_pushback = collision_result / num_wheels as f32;
            }
        }

        depth
    }

    fn calc_friction_impulses(&mut self, time_step: f32) {
        let cb = self.chassis_body.borrow();
        let cb_co = cb.collision_object.borrow();
        let friction_scale = cb.get_mass() / 3.0;

        for wheel in &mut self.wheel_info {
            let Some(ground_rb_ref) = wheel.wheel_info.raycast_info.ground_object.as_ref() else {
                wheel.impulse = Vec3A::ZERO;
                continue;
            };
            let ground_rb = ground_rb_ref.borrow();

            let mut axle_dir = wheel
                .wheel_info
                .world_transform
                .matrix3
                .col(self.index_right_axis);

            let surf_normal_ws = wheel.wheel_info.raycast_info.contact_normal_ws;
            let proj = axle_dir.dot(surf_normal_ws);
            axle_dir -= surf_normal_ws * proj;
            axle_dir = axle_dir.normalize_or_zero();

            let forward_dir = surf_normal_ws.cross(axle_dir).normalize_or_zero();

            let side_impulse = resolve_single_bilateral(
                &cb,
                wheel.wheel_info.raycast_info.contact_point_ws,
                &ground_rb,
                wheel.wheel_info.raycast_info.contact_point_ws,
                axle_dir,
            );

            let rolling_friction = if wheel.wheel_info.engine_force == 0.0 {
                if wheel.wheel_info.brake == 0.0 {
                    0.0
                } else {
                    const ROLLING_FRICTION_SCALE: f32 = 113.73963;

                    let contact_point = wheel.wheel_info.raycast_info.contact_point_ws;
                    let car_rel_contact_point =
                        contact_point - cb_co.get_world_transform().translation;

                    let v1 = cb.get_velocity_in_local_point(car_rel_contact_point);
                    let v2 = ground_rb.get_velocity_in_local_point(car_rel_contact_point);
                    let contact_vel = v1 - v2;
                    let mut rel_vel = contact_vel.dot(forward_dir);

                    if time_step > 1.0 / 80.0 {
                        let threshold = 0.8 - (1.0 / (time_step * 150.0));
                        if rel_vel.abs() < threshold {
                            rel_vel = 0.0;
                        }
                    }

                    (-rel_vel * ROLLING_FRICTION_SCALE)
                        .clamp(-wheel.wheel_info.brake, wheel.wheel_info.brake)
                }
            } else {
                -wheel.wheel_info.engine_force / friction_scale
            };

            let total_friciton_force = forward_dir * rolling_friction * wheel.long_friction
                + axle_dir * side_impulse * wheel.lat_friction;
            wheel.impulse = total_friciton_force * friction_scale;
        }
    }

    pub fn update_vehicle_first(&mut self, collision_world: &DiscreteDynamicsWorld, step: f32) {
        for i in 0..self.wheel_info.len() {
            self.update_wheel_transform(i);
        }

        // simulate suspension
        for i in 0..self.wheel_info.len() {
            self.ray_cast(collision_world, i);
        }

        self.calc_friction_impulses(step);
    }

    fn update_suspension(&mut self, delta_time: f32) {
        for wheel in &mut self.wheel_info {
            if !wheel.wheel_info.raycast_info.is_in_contact {
                wheel.wheel_info.wheels_suspension_force = 0.0;
                continue;
            }

            let force = (wheel.wheel_info.suspension_rest_length_1
                - wheel.wheel_info.raycast_info.suspension_length)
                * wheel.wheel_info.suspension_stiffness
                * wheel.wheel_info.clipped_inv_contact_dot_suspension;
            let damping_vel_scale = if wheel.wheel_info.suspension_relative_velcity < 0.0 {
                wheel.wheel_info.wheels_damping_compression
            } else {
                wheel.wheel_info.wheels_damping_relaxation
            };

            wheel.wheel_info.wheels_suspension_force =
                force - (damping_vel_scale * wheel.wheel_info.suspension_relative_velcity);
            wheel.wheel_info.wheels_suspension_force *= wheel.suspsension_force_scale;
            wheel.wheel_info.wheels_suspension_force =
                wheel.wheel_info.wheels_suspension_force.max(0.0);

            if wheel.wheel_info.wheels_suspension_force == 0.0 {
                continue;
            }

            let mut rb = self.chassis_body.borrow_mut();
            let contact_point_offset = wheel.wheel_info.raycast_info.contact_point_ws
                - rb.collision_object
                    .borrow()
                    .get_world_transform()
                    .translation;
            let base_force_scale =
                wheel.wheel_info.wheels_suspension_force * delta_time + wheel.extra_pushback;
            let force = wheel.wheel_info.raycast_info.contact_normal_ws * base_force_scale;
            rb.apply_impulse(force, contact_point_offset);
        }
    }

    fn apply_friction_impulses(&mut self, time_step: f32) {
        let mut rb = self.chassis_body.borrow_mut();
        let trans = *rb.collision_object.borrow().get_world_transform();

        let up_dir = trans.matrix3.col(self.index_up_axis);

        for wheel in &mut self.wheel_info {
            if wheel.impulse == Vec3A::ZERO {
                continue;
            }

            let wheel_contact_offset =
                wheel.wheel_info.raycast_info.contact_point_ws - trans.translation;
            let contact_up_dot = up_dir.dot(wheel_contact_offset);
            let wheel_rel_pos = wheel_contact_offset - up_dir * contact_up_dot;
            rb.apply_impulse(wheel.impulse * time_step, wheel_rel_pos);
        }
    }

    pub fn update_vehicle_second(&mut self, step: f32) {
        self.update_suspension(step);
        self.apply_friction_impulses(step);
    }
}
