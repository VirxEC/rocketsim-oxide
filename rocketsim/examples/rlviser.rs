use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use rocketsim::{
    Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default, rlviser::RLViser,
};

fn main() {
    init_from_default(true).unwrap();
    let mut rlviser = RLViser::new().unwrap();
    let mut arena = Arena::new(GameMode::Soccar);

    let ids = [
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE),
        arena.add_car(Team::Orange, CarBodyConfig::OCTANE),
    ];

    arena.reset_to_random_kickoff();

    let mut render_interval = Duration::from_secs_f32(1.0 / 120.);
    let mut next_time = Instant::now() + render_interval;

    // run for 300 seconds
    for _ in 0..(120 * 3000) {
        for id in ids {
            arena.car_mut(id).set_controls(CarControls {
                throttle: 1.0,
                boost: true,
                ..Default::default()
            });
        }

        // step the arena
        arena.step(1);

        // get the full game state
        let game_state = arena.get_game_state();

        // send it to rlviser
        rlviser.send_state(&game_state).unwrap();

        loop {
            // handle rlviser state setting (+ game speed & paused)
            rlviser
                .handle_state_settings(&mut arena, &mut render_interval)
                .unwrap();

            // ensure we only run at the requested game speed
            let wait_time = next_time - Instant::now();
            if wait_time > Duration::default() {
                sleep(wait_time);
            }
            next_time += render_interval;

            // If we were told to pause,
            // make sure to keep reading rlviser state
            // until we are unpaused
            //
            // reuse the real-time sleep interval
            // while paused to prevent busy-waiting
            if !rlviser.is_paused() {
                break;
            }
        }
    }

    // Close rlviser
    rlviser.quit().unwrap();
}
