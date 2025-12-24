#![cfg(feature = "rlviser")]

use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, CarConfig, GameMode, Team, init_from_default, rlviser::RLViser};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);

    let car = arena.car_mut(id);
    let mut state = *car.get_state();
    let f = Vec3A::new(0., -1., 0.5).normalize();
    let up = Vec3A::NEG_Z;
    let tr = up.cross(f);
    let u = f.cross(tr).normalize();
    let r = u.cross(f).normalize();
    state.phys.rot_mat = Mat3A::from_cols(f, r, u);

    state.phys.pos.x = 2000.0;
    state.phys.pos.y = 4960.0;
    state.phys.pos.z = 40.0;
    arena.set_car_state(id, state);

    arena.step(1);

    let state = arena.car_mut(id).get_state();
    println!("\npos: {}", state.phys.pos);
    println!("vel: {}", state.phys.vel);
    println!("ang_vel: {}", state.phys.ang_vel);

    let mut rlviser = RLViser::new().unwrap();
    rlviser.send_state(&arena.get_game_state()).unwrap();
}
