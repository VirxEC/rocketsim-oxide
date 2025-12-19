use std::time::Instant;

use rocketsim::{
    GameMode, init_from_default,
    Arena, BallState
};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let start = Instant::now();
    for _ in 0..20_000 {
        let mut ball = BallState::DEFAULT;
        ball.physics.vel.x = 600.0;
        ball.physics.vel.y = 1550.0;
        ball.physics.vel.z = 0.0;

        arena.set_ball(ball);
        arena.step(720);
    }
    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    println!(
        "Elapsed: {elapsed}\nTPS: {}",
        (20_000 * 720) as f32 / elapsed
    );
}
