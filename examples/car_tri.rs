use rocketsim::{
    GameMode, init_from_default,
    rlviser::RLViser,
    sim::{Arena, CarConfig, Team},
};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);

    let car = arena.get_car_mut(id).unwrap();
    // car.controls.throttle = 1.0;
    let mut state = *car.get_state();
    state.physics.pos.x = 2000.0;
    state.physics.pos.y = 4925.0;
    state.physics.pos.z = 17.0;
    arena.set_car_state(id, state);

    arena.step(1);

    let state = arena.get_car_mut(id).unwrap().get_state();
    println!("\npos: {}", state.physics.pos);
    println!("vel: {}", state.physics.vel);
    println!("ang_vel: {}", state.physics.ang_vel);

    let mut rlviser = RLViser::new().unwrap();
    rlviser.send_state(&arena.get_game_state()).unwrap();
}
