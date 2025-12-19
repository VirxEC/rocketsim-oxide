use rocketsim::{
    GameMode, init_from_default,
    sim::{Arena, BallState},
};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let mut ball = BallState::DEFAULT;
    ball.physics.vel.x = 600.0;
    ball.physics.vel.y = 1550.0;
    ball.physics.vel.z = 0.0;

    arena.set_ball(ball);
    arena.step(720);

    let ball = arena.get_ball();
    println!("\npos: {}", ball.physics.pos);
    println!("vel: {}", ball.physics.vel);
    println!("ang_vel: {}", ball.physics.ang_vel);
}
