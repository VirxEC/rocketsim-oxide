use std::time::Instant;

use rocketsim::{
    GameMode, init_from_default,
    Arena, CarConfig, Team
};

fn main() {
    fastrand::seed(4);
    init_from_default(true).unwrap();

    let start = Instant::now();
    let mut arena = Arena::new(GameMode::Soccar);
    println!(
        "Finished initializing Arena in {:.4}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);
    let id2 = arena.add_car(Team::Orange, CarConfig::OCTANE);
    arena.reset_to_random_kickoff();

    let mut ball_state = *arena.get_ball();
    ball_state.physics.pos.z += 200.;
    arena.set_ball(ball_state);

    {
        let car1 = arena.get_car(id).unwrap();
        let mut state1 = *car1.get_state();
        state1.physics.pos.y = -2000.0;
        arena.set_car_state(id, state1);
        dbg!(state1.physics.pos);

        let car2 = arena.get_car_mut(id2).unwrap();
        car2.controls.throttle = 1.0;
        let mut state2 = *car2.get_state();
        state2.physics.pos.y = -1500.0;
        state2.physics.vel.y = -1500.0;
        dbg!(state2.physics.pos);
        arena.set_car_state(id, state2);
    };

    let start = Instant::now();
    arena.step(60);
    println!(
        "Stepped Arena in {}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let state = arena.get_car(id).unwrap().get_state();
    println!("\npos: {}", state.physics.pos);
    println!("vel: {}", state.physics.vel);
    println!("ang_vel: {}", state.physics.ang_vel);
    println!("rot_mat: {}", state.physics.rot_mat);
    println!("is_demoed: {:?}", state.is_demoed);
    println!("car_contact: {:?}", state.car_contact);
    println!("wheels_with_contact: {:?}", state.wheels_with_contact);

    let state = arena.get_car(id2).unwrap().get_state();
    println!("\npos: {}", state.physics.pos);
    println!("vel: {}", state.physics.vel);
    println!("ang_vel: {}", state.physics.ang_vel);
    println!("rot_mat: {}", state.physics.rot_mat);
    println!("is_demoed: {:?}", state.is_demoed);
    println!("car_contact: {:?}", state.car_contact);
    println!("wheels_with_contact: {:?}", state.wheels_with_contact);
}
