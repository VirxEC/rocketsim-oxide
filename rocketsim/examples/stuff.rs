use rocketsim::{GameMode, init_from_default, sim::Arena};
use std::time::Instant;

fn main() {
    init_from_default(false).unwrap();

    let start = Instant::now();
    let mut arena = Arena::new(GameMode::Soccar);
    println!(
        "Finished initializing Arena in {:.4}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let mut ball = arena.ball.get_state();
    ball.physics.vel.z = 500.0;

    arena.ball.set_state(ball);

    let start = Instant::now();
    arena.step(440);
    println!(
        "Stepped Arena in {}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );

    let ball = arena.ball.get_state();
    dbg!(ball.physics.pos);
    dbg!(ball.physics.vel);
}
