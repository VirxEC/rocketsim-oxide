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
        ball.physics.vel.x = 600.0;
        ball.physics.vel.y = 1550.0;
        ball.physics.vel.z = 0.0;

        arena.ball.set_state(ball);
        arena.step(720);
    }
    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    println!(
        "Elapsed: {elapsed}\nSPS: {}",
        (10000 * 720) as f32 / elapsed
    );
}
