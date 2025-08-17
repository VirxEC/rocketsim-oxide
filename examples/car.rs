use glam::{Mat3A, Vec3A};
use rocketsim::{
    GameMode, init_from_default,
    sim::{Arena, CarConfig, Team},
};
use std::time::Instant;

fn main() {
    init_from_default(true).unwrap();

    let mut arena = Arena::new(GameMode::Soccar);
    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);

    fastrand::seed(0);
    arena.reset_to_random_kickoff();

    let mut ball_state = arena.objects.ball.get_state();
    ball_state.physics.pos.z += 1000.;
    // ball_state.physics.vel.z = -10.;
    arena.objects.ball.set_state(ball_state);

    let state = {
        let car = arena.objects.cars.get_mut(&id).unwrap();
        let mut state = car.get_state();
        state.physics.pos.z = 43.0;
        state.is_on_ground = false;

        let f = Vec3A::new(1., 1., 1.).normalize();
        let up = Vec3A::Z;
        let tr = up.cross(f);
        let u = f.cross(tr).normalize();
        let r = u.cross(f).normalize();
        state.physics.rot_mat = Mat3A::from_cols(f, r, u);
        state
    };

    let start = Instant::now();
    for _ in 0..10_000 {
        arena.objects.ball.set_state(ball_state);
        arena.objects.cars.get_mut(&id).unwrap().set_state(state);
        arena.step(720);
    }
    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    println!(
        "Elapsed: {elapsed}\nSPS: {}",
        (10_000 * 720) as f32 / elapsed
    );
}
