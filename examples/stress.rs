use std::time::Instant;

use glam::{Mat3A, Vec3A};
use rocketsim::{
    GameMode, init_from_default,
    sim::{Arena, ArenaConfig, CarConfig, Team},
};

const NUM_CARS: u8 = 8;

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new_with_config(
        GameMode::Soccar,
        ArenaConfig {
            rng_seed: Some(0),
            ..Default::default()
        },
        120,
    );

    let mut ids = Vec::new();
    for i in 0..NUM_CARS {
        let id = arena.add_car(Team::try_from(i % 2).unwrap(), CarConfig::OCTANE);
        ids.push(id);
    }

    arena.reset_to_random_kickoff();

    let mut ball_state = *arena.get_ball();
    ball_state.physics.vel.x = 600.0;
    ball_state.physics.vel.y = 1550.0;
    ball_state.physics.vel.z = 0.0;
    arena.set_ball(ball_state);

    let mut states = Vec::new();
    for &id in &ids {
        let car = arena.get_car_mut(id).unwrap();
        car.controls.throttle = 1.0;

        let mut state = *car.get_state();
        state.physics.pos.z = 43.0;
        state.is_on_ground = false;

        let f = Vec3A::new(1., 1., 1.).normalize();
        let up = Vec3A::Z;
        let tr = up.cross(f);
        let u = f.cross(tr).normalize();
        let r = u.cross(f).normalize();
        state.physics.rot_mat = Mat3A::from_cols(f, r, u);
        states.push(state);
    }

    let start = Instant::now();
    for _ in 0..1_000 {
        arena.set_ball(ball_state);
        for (&id, &state) in ids.iter().zip(&states) {
            arena.set_car_state(id, state);
        }

        arena.step(720);
    }
    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    println!(
        "Elapsed: {elapsed}\nTPS: {}",
        (1_000 * 720) as f32 / elapsed
    );
}
