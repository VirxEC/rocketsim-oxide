use super::{
    raycaster::VehicleRaycaster,
    wheel_info::{WheelInfo, WheelInfoConstructionInfo},
};
use crate::bullet::{
    collision::dispatch::collision_world::CollisionWorld,
    dynamics::{action_interface::ActionInterface, rigid_body::RigidBody},
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

impl ActionInterface for VehicleRL {
    fn update_action(&mut self, collision_world: &mut CollisionWorld, delta_time_step: f32) {
        todo!()
    }
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
            wheel_info: const { ArrayVec::new_const() },
        }
    }

    fn update_wheel_transform(&mut self, wheel: &mut WheelInfoRL) {
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

        let mut wheel = WheelInfoRL::new(ci);
        self.update_wheel_transform(&mut wheel);
        self.wheel_info.push(wheel);
    }
}
