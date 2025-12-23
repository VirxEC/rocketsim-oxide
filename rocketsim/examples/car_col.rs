use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, CarConfig, GameMode, Team, init_from_default};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let id = arena.add_car(Team::Blue, CarConfig::OCTANE);
    let id2 = arena.add_car(Team::Orange, CarConfig::OCTANE);

    let mut ball_state = *arena.get_ball();
    ball_state.phys.pos.z += 200.0;
    arena.set_ball(ball_state);

    {
        let car1 = arena.get_car(id).unwrap();
        let mut state1 = *car1.get_state();
        state1.phys.pos.x = 500.0;
        state1.phys.pos.y = -74.0;
        state1.phys.pos.z = 17.0;
        state1.phys.rot_mat = Mat3A::from_cols(Vec3A::Y, Vec3A::NEG_X, Vec3A::Z);
        arena.set_car_state(id, state1);

        let car2 = arena.get_car_mut(id2).unwrap();
        let mut state2 = *car2.get_state();
        state2.phys.rot_mat = Mat3A::from_cols(Vec3A::NEG_Y, Vec3A::X, Vec3A::Z);
        state2.phys.pos.x = 500.0;
        state2.phys.pos.y = 74.0;
        state2.phys.pos.z = 17.0;
        state2.phys.vel.y = -1500.0;
        state2.controls.throttle = 1.0;
        arena.set_car_state(id2, state2);
    };

    arena.step(1);

    let state = arena.get_car(id).unwrap().get_state();
    println!("\npos: {}", state.phys.pos);
    println!("vel: {}", state.phys.vel);
    println!("ang_vel: {}", state.phys.ang_vel);
    println!("rot_mat: {}", state.phys.rot_mat);
    println!("is_demoed: {:?}", state.is_demoed);
    println!("car_contact: {:?}", state.car_contact);

    let state = arena.get_car(id2).unwrap().get_state();
    println!("\npos: {}", state.phys.pos);
    println!("vel: {}", state.phys.vel);
    println!("ang_vel: {}", state.phys.ang_vel);
    println!("rot_mat: {}", state.phys.rot_mat);
    println!("is_demoed: {:?}", state.is_demoed);
    println!("car_contact: {:?}", state.car_contact);
}
