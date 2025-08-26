use rocketsim::{
    GameMode, init_from_default,
    sim::{Arena, CarConfig, Team},
};
use std::time::Instant;

fn main() {
    init_from_default(false).unwrap();

    let start = Instant::now();
    let mut arena = Arena::new(GameMode::Soccar);
    println!(
        "Finished initializing Arena in {:.4}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    fastrand::seed(0);
    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);
    arena.reset_to_random_kickoff();

    let car = arena.objects.cars.get_mut(&id).unwrap();
    println!("pos: {}", car.get_state().physics.pos);
    car.controls.throttle = 1.0;
    car.controls.boost = false;

    let start = Instant::now();
    arena.step(468);
    println!(
        "Stepped Arena in {}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let state = arena.objects.cars.get(&id).unwrap().get_state();
    println!("\npos: {}", state.physics.pos);
    println!("vel: {}", state.physics.vel);
    println!("ang_vel: {}", state.physics.ang_vel);

    let ball = arena.objects.ball.get_state();
    println!("\npos: {}", ball.physics.pos);
    println!("vel: {}", ball.physics.vel);
    println!("ang_vel: {}", ball.physics.ang_vel);
}
