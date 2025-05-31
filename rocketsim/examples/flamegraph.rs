use std::time::Instant;

use rocketsim::{
    GameMode, init_from_default,
    sim::{Arena, BallState},
};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let start = Instant::now();
    for _ in 0..10_000 {
        let mut ball = BallState::DEFAULT;
        ball.physics.vel.x = 500.0;
        ball.physics.vel.y = 500.0;
        ball.physics.vel.z = 500.0;

        arena.ball.set_state(ball);
        arena.step(440);
    }
    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    println!(
        "Elapsed: {elapsed}\nSPS: {}",
        (10000 * 440) as f32 / elapsed
    );
}
