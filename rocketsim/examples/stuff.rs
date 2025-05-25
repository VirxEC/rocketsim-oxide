use rocketsim::{GameMode, init_from_default, sim::Arena};
use std::time::Instant;

fn main() {
    init_from_default(false).unwrap();

    let start = Instant::now();
    let arena = Arena::new(GameMode::Soccar);
    println!(
        "Finished initializing Arena in {:.3}s!",
        Instant::now().duration_since(start).as_secs_f32()
    );
    std::hint::black_box(arena);

    // arena.steps(20000000);
}
