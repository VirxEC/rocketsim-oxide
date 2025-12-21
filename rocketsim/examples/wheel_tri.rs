use rocketsim::{Arena, CarConfig, GameMode, Team, init_from_default};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);

    let car = arena.get_car_mut(id).unwrap();
    // car.controls.throttle = 1.0;
    let mut state = *car.get_state();
    state.phys.pos.x = 2000.0;
    state.phys.pos.y = 4925.0;
    state.phys.pos.z = 17.0;
    arena.set_car_state(id, state);

    arena.step(1);

    let state = arena.get_car_mut(id).unwrap().get_state();
    println!("\npos: {}", state.phys.pos);
    println!("vel: {}", state.phys.vel);
    println!("ang_vel: {}", state.phys.ang_vel);
}
