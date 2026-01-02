use std::time::Instant;

use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};

fn main() {
    init_from_default(true).unwrap();

    let start = Instant::now();
    let mut arena = Arena::new(GameMode::Soccar);
    println!(
        "Finished initializing Arena in {:.4}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    fastrand::seed(0);
    let idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let idx2 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff();

    let mut ball_state = *arena.get_ball_state();
    ball_state.phys.pos.z += 200.0;
    arena.set_ball_state(ball_state);

    {
        let state = arena.get_car(idx).get_state();

        let car = arena.get_car(idx2);
        let mut state2 = *car.get_state();
        state2.phys.pos = state.phys.pos;
        state2.phys.pos.z = 100.0;
        state2.is_on_ground = false;

        arena.set_car_state(idx2, state2);
    };

    let start = Instant::now();
    arena.step(720);
    println!(
        "Stepped Arena in {}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let state = arena.get_car(idx).get_state();
    println!("\npos: {}", state.phys.pos);
    println!("vel: {}", state.phys.vel);
    println!("ang_vel: {}", state.phys.ang_vel);
    println!("rot_mat: {}", state.phys.rot_mat);
    println!("wheels_with_contact: {:?}", state.wheels_with_contact);

    let state = arena.get_car(idx2).get_state();
    println!("\npos: {}", state.phys.pos);
    println!("vel: {}", state.phys.vel);
    println!("ang_vel: {}", state.phys.ang_vel);
    println!("rot_mat: {}", state.phys.rot_mat);
    println!("wheels_with_contact: {:?}", state.wheels_with_contact);
}
