use rocketsim::{Arena, BallState, GameMode, init_from_default};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let mut ball = BallState::DEFAULT;
    ball.phys.vel.x = 600.0;
    ball.phys.vel.y = 1550.0;
    ball.phys.vel.z = 0.0;

    arena.set_ball(ball);
    arena.step(720);

    let ball = arena.get_ball();
    println!("\npos: {}", ball.phys.pos);
    println!("vel: {}", ball.phys.vel);
    println!("ang_vel: {}", ball.phys.ang_vel);
}
