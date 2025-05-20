use rocketsim::init_from_default;
use std::io::Result as IoResult;

fn main() -> IoResult<()> {
    for _ in 0..200 {
        std::hint::black_box(init_from_default(true)?);
    }

    Ok(())
}
