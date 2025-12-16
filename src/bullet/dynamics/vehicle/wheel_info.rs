use std::{cell::RefCell, rc::Rc};

use glam::{Affine3A, Vec3A};

use crate::bullet::dynamics::rigid_body::RigidBody;

#[derive(Clone, Copy)]
pub struct WheelInfoConstructionInfo {
    pub chassis_connection_cs: Vec3A,
    pub wheel_direction_cs: Vec3A,
    pub wheel_axle_cs: Vec3A,
    pub suspension_rest_length: f32,
    pub max_suspension_travel_cm: f32,
    pub wheel_radius: f32,
    pub suspension_stiffness: f32,
    pub wheels_damping_compression: f32,
    pub wheels_damping_relaxation: f32,
    // pub friction_slip: f32,
    // pub max_suspension_force: f32,
    // pub is_front_wheel: bool,
}

#[derive(Default)]
pub struct RaycastInfo {
    pub contact_normal_ws: Vec3A,
    pub contact_point_ws: Vec3A,
    pub suspension_length: f32,
    pub hard_point_ws: Vec3A,
    pub wheel_direction_ws: Vec3A,
    pub wheel_axle_ws: Vec3A,
    pub is_in_contact: bool,
    pub ground_object: Option<Rc<RefCell<RigidBody>>>,
}

pub struct WheelInfo {
    pub raycast_info: RaycastInfo,
    pub world_transform: Affine3A,
    pub chassis_connection_point_cs: Vec3A,
    pub wheel_direction_cs: Vec3A,
    pub wheel_axle_cs: Vec3A,
    pub suspension_rest_length_1: f32,
    pub max_suspension_travel_cm: f32,
    pub wheels_radius: f32,
    pub suspension_stiffness: f32,
    pub wheels_damping_compression: f32,
    pub wheels_damping_relaxation: f32,
    // pub friction_slip: f32,
    // pub steering: f32,
    // pub rotation: f32,
    // pub delta_rotation: f32,
    // pub roll_influence: f32,
    // pub max_suspension_force: f32,
    pub engine_force: f32,
    pub brake: f32,
    // pub is_front_wheel: bool,
    // void* m_clientInfo;
    pub clipped_inv_contact_dot_suspension: f32,
    pub suspension_relative_velcity: f32,
    pub wheels_suspension_force: f32,
    // pub skid_info: f32,
}

impl WheelInfo {
    pub fn new(ci: WheelInfoConstructionInfo) -> Self {
        Self {
            suspension_rest_length_1: ci.suspension_rest_length,
            max_suspension_travel_cm: ci.max_suspension_travel_cm,
            wheels_radius: ci.wheel_radius,
            suspension_stiffness: ci.suspension_stiffness,
            wheels_damping_compression: ci.wheels_damping_compression,
            wheels_damping_relaxation: ci.wheels_damping_relaxation,
            chassis_connection_point_cs: ci.chassis_connection_cs,
            wheel_direction_cs: ci.wheel_direction_cs,
            wheel_axle_cs: ci.wheel_axle_cs,
            // friction_slip: ci.friction_slip,
            // steering: 0.0,
            engine_force: 0.0,
            // rotation: 0.0,
            // delta_rotation: 0.00,
            brake: 0.0,
            // roll_influence: 0.1,
            // is_front_wheel: ci.is_front_wheel,
            // max_suspension_force: ci.max_suspension_force,
            raycast_info: RaycastInfo::default(),
            world_transform: Affine3A::IDENTITY,
            clipped_inv_contact_dot_suspension: 0.0,
            suspension_relative_velcity: 0.0,
            wheels_suspension_force: 0.0,
            // skid_info: 0.0,
        }
    }
}
