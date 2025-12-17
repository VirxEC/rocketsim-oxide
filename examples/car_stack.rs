use std::time::Instant;

use rocketsim::{
    GameMode, init_from_default,
    sim::{Arena, CarConfig, Team},
};

fn main() {
    init_from_default(true).unwrap();

    let start = Instant::now();
    let mut arena = Arena::new(GameMode::Soccar);
    println!(
        "Finished initializing Arena in {:.4}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    fastrand::seed(0);
    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);
    let id2 = arena.add_car(Team::Blue, CarConfig::OCTANE);
    arena.reset_to_random_kickoff();

    let mut ball_state = *arena.get_ball();
    ball_state.physics.pos.z += 200.;
    arena.set_ball(ball_state);

    {
        let state = arena.get_car(id).unwrap().get_state();

        let car = arena.get_car(id2).unwrap();
        let mut state2 = *car.get_state();
        state2.physics.pos = state.physics.pos;
        state2.physics.pos.z = 100.0;
        state2.is_on_ground = false;

        arena.set_car_state(id2, state2);
    };

    let start = Instant::now();
    arena.step(720);
    println!(
        "Stepped Arena in {}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let state = arena.get_car(id).unwrap().get_state();
    println!("\npos: {}", state.physics.pos);
    println!("vel: {}", state.physics.vel);
    println!("ang_vel: {}", state.physics.ang_vel);
    println!("rot_mat: {}", state.physics.rot_mat);
    println!("wheels_with_contact: {:?}", state.wheels_with_contact);

    let state = arena.get_car(id2).unwrap().get_state();
    println!("\npos: {}", state.physics.pos);
    println!("vel: {}", state.physics.vel);
    println!("ang_vel: {}", state.physics.ang_vel);
    println!("rot_mat: {}", state.physics.rot_mat);
    println!("wheels_with_contact: {:?}", state.wheels_with_contact);
}
