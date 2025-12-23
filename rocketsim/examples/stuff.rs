use std::time::Instant;

use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarConfig, GameMode, Team, init_from_default, CarControls};

fn hex_vec3a(v: Vec3A) -> String {
    format!("{:x?}", v.to_array().map(|f| f.to_bits()))
}

fn main() {
    init_from_default(false).unwrap();

    let start = Instant::now();
    let mut arena = Arena::new_with_config(
        GameMode::Soccar,
        ArenaConfig {
            rng_seed: Some(11),
            ..Default::default()
        },
        120,
    );
    println!(
        "Finished initializing Arena in {:.4}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);
    arena.reset_to_random_kickoff();

    let car = arena.get_car_mut(id).unwrap();
    println!("pos: {}", car.get_state().phys.pos);
    let mut car_controls = CarControls::DEFAULT;
    car_controls.throttle = 1.0;
    car_controls.boost = false;
    car.set_controls(car_controls);

    for i in 0..3 {
        println!("\nstep {i}");
        arena.step(1);

        let state = arena.get_car(id).unwrap().get_state();
        // println!("\npos: {}", state.physics.pos);
        // println!("vel: {}", state.physics.vel);
        // println!("ang_vel: {}", state.physics.ang_vel);
        println!("\npos: {}", hex_vec3a(state.phys.pos));
        println!("vel: {}", hex_vec3a(state.phys.vel));
        println!("ang_vel: {}", hex_vec3a(state.phys.ang_vel));
        println!("forward: {}", hex_vec3a(state.phys.rot_mat.x_axis));
        println!("right: {}", hex_vec3a(state.phys.rot_mat.y_axis));
        println!("up: {}", hex_vec3a(state.phys.rot_mat.z_axis));

        let ball = arena.get_ball();
        // println!("\npos: {}", ball.physics.pos);
        // println!("vel: {}", ball.physics.vel);
        // println!("ang_vel: {}", ball.physics.ang_vel);
        println!("\npos: {}", hex_vec3a(ball.phys.pos));
        println!("vel: {}", hex_vec3a(ball.phys.vel));
        println!("ang_vel: {}", hex_vec3a(ball.phys.ang_vel));
    }
}
