use rocketsim::init_from_default;
use std::io::Result as IoResult;

fn main() -> IoResult<()> {
    init_from_default(false)?;

    // let mut arena = Arena::default();
    // arena.steps(20000000);

    Ok(())
}
