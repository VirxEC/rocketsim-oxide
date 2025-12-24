use glam::{EulerRot, IVec3, Mat3A};
use rocketsim::{CarControls, GameMode, Team};

use crate::comparison_test::{BallSetup, CarSetup, ControlsBuilder, TestCase};

pub fn make_ball_cases() -> Vec<TestCase> {
    let simple_ball_case = |name: &'static str,
                            duration_ticks: usize,
                            pos: (i32, i32, i32),
                            vel: (i32, i32, i32),
                            ang_vel: (i32, i32, i32)|
     -> TestCase {
        TestCase {
            name,
            game_mode: GameMode::Soccar,
            car_setups: Vec::new(),
            ball_setup: Some(
                BallSetup::new(IVec3::new(pos.0, pos.1, pos.2).as_vec3a())
                    .with_vel(IVec3::new(vel.0, vel.1, vel.2).as_vec3a())
                    .with_ang_vel(IVec3::new(ang_vel.0, ang_vel.1, ang_vel.2).as_vec3a()),
            ),
            duration_ticks,
        }
    };

    vec![
        simple_ball_case(
            "bounce_ground",
            30,
            (0, 0, 100),
            (200, 300, -600),
            (0, 0, 0),
        ),
        simple_ball_case(
            "bounce_ground_ang_vel",
            30,
            (0, 0, 100),
            (200, 300, -600),
            (3, 4, 5),
        ),
        simple_ball_case(
            "bounce_crazy",
            240,
            (2000, 1000, 1000),
            (2000, 4000, -3000),
            (1, 2, 3),
        ),
        simple_ball_case(
            "roll_up_slope",
            80,
            (3800, 0, 93),
            (3000, 200, 0),
            (0, 0, 0),
        ),
        simple_ball_case(
            "bounce_off_slope",
            30,
            (3800, 0, 200),
            (1000, 100, -500),
            (0, 0, 0),
        ),
        simple_ball_case(
            "into_goal",
            120,
            (0, 4000, 500),
            (100, 3500, -30),
            (0, 0, 0),
        ),
        simple_ball_case(
            "crossbar_down",
            60,
            (-150, 4000, 650),
            (150, 3500, -30),
            (0, 0, 0),
        ),
        simple_ball_case(
            "post_out",
            60,
            (-890, 4000, 200),
            (-20, 3500, 50),
            (0, 0, 0),
        ),
    ]
}

///////////////////

pub fn make_car_cases() -> Vec<TestCase> {
    let simple_car_case = |name: &'static str,
                           duration_ticks: usize,
                           pos: (i32, i32, i32),
                           euler_rot_ypr: (f32, f32, f32),
                           vel: (i32, i32, i32),
                           ang_vel: (i32, i32, i32),
                           car_controls: CarControls,
                           is_on_ground: bool|
     -> TestCase {
        TestCase {
            name,
            game_mode: GameMode::Soccar,
            car_setups: vec![
                CarSetup::new(Team::Blue, IVec3::new(pos.0, pos.1, pos.2).as_vec3a())
                    .with_rot(Mat3A::from_euler(
                        EulerRot::ZYX,
                        euler_rot_ypr.0,
                        euler_rot_ypr.1,
                        euler_rot_ypr.2,
                    ))
                    .with_vel(IVec3::new(vel.0, vel.1, vel.2).as_vec3a())
                    .with_ang_vel(IVec3::new(ang_vel.0, ang_vel.1, ang_vel.2).as_vec3a())
                    .with_controls(car_controls)
                    .with_on_ground(is_on_ground),
            ],
            ball_setup: None,
            duration_ticks,
        }
    };

    vec![
        simple_car_case(
            "drive_forward",
            30,
            (0, 0, 17),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new().with_throttle(1.0).build(),
            true,
        ),
        simple_car_case(
            "drive_forward_boost",
            30,
            (0, 0, 17),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new()
                .with_throttle(1.0)
                .with_boost(true)
                .build(),
            true,
        ),
        simple_car_case(
            "drive_forward_turn",
            30,
            (0, 0, 17),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new()
                .with_throttle(1.0)
                .with_steer(1.0)
                .build(),
            true,
        ),
        simple_car_case(
            "drive_forward_turn_handbrake",
            30,
            (0, 0, 17),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new()
                .with_throttle(1.0)
                .with_steer(1.0)
                .with_handbrake(true)
                .build(),
            true,
        ),
        simple_car_case(
            "drive_up_slope",
            60,
            (3700, 0, 17),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new().with_throttle(1.0).build(),
            true,
        ),
        simple_car_case(
            "jump_long_stationary",
            30,
            (0, 0, 17),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new().with_jump(true).build(),
            true,
        ),
        simple_car_case(
            "land_ground_simple",
            30,
            (0, 0, 40),
            (0.0, 0.0, 0.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new().build(),
            false,
        ),
        simple_car_case(
            "land_ground_complex",
            60,
            (0, 0, 100),
            (0.17, -0.044, 0.28),
            (100, -50, -300),
            (0, 0, 0),
            ControlsBuilder::new().build(),
            false,
        ),
        simple_car_case(
            "pogo",
            120,
            (0, 0, 250),
            (-2.0952558, 0.7192848, 0.8505284),
            (0, 0, -2100),
            (0, 0, 0),
            ControlsBuilder::new().build(),
            false,
        ),
        simple_car_case(
            "double_jump",
            2,
            (0, 0, 500),
            (1.0, 2.0, 3.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new().with_jump(true).build(),
            true,
        ),
        simple_car_case(
            "flip_forward",
            120,
            (0, 0, 500),
            (1.0, 2.0, 3.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new()
                .with_pitch(-1.0)
                .with_jump(true)
                .build(),
            false,
        ),
        simple_car_case(
            "flip_partial_input",
            120,
            (0, 0, 500),
            (1.0, 2.0, 3.0),
            (0, 0, 0),
            (0, 0, 0),
            ControlsBuilder::new()
                .with_pitch(-0.3)
                .with_yaw(-0.7)
                .with_roll(0.8)
                .with_jump(true)
                .build(),
            false,
        ),
    ]
}
