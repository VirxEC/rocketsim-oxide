use rocketsim::{GameMode, init_from_default, sim::Arena};
use std::hint::black_box;

fn main() {
    init_from_default(true).unwrap();

    for _ in 0..500 {
        black_box(Arena::new(GameMode::Soccar));
    }
}
