use std::io;

use bullet_rs::{arena::Arena, collision_mesh_file::COLLISION_MESH_BASE_PATH, init};

fn main() -> io::Result<()> {
    init(COLLISION_MESH_BASE_PATH)?;

    let mut arena = Arena::default();
    arena.steps(20000000);

    Ok(())
}
