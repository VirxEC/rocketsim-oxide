use std::time::Instant;

use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, CarConfig, GameMode, Team, init_from_default};

fn main() {
    init_from_default(false).unwrap();

    let mut arena = Arena::new(GameMode::Soccar);
    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);

    fastrand::seed(0);
    arena.reset_to_random_kickoff();

    let mut ball_state = *arena.get_ball();
    ball_state.phys.pos.z += 1000.0;
    arena.set_ball(ball_state);

    let state = {
        let car = arena.get_car(id).unwrap();
        let mut state = *car.get_state();
        state.phys.pos.z = 43.0;
        state.is_on_ground = false;

        let f = Vec3A::new(1., 1., 1.).normalize();
        let up = Vec3A::Z;
        let tr = up.cross(f);
        let u = f.cross(tr).normalize();
        let r = u.cross(f).normalize();
        state.phys.rot_mat = Mat3A::from_cols(f, r, u);
        state
    };

    let start = Instant::now();
    for _ in 0..10_000 {
        arena.set_ball(ball_state);
        arena.set_car_state(id, state);
        arena.step(720);
    }
    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    println!(
        "Elapsed: {elapsed}\nTPS: {}",
        (10_000 * 720) as f32 / elapsed
    );
}
