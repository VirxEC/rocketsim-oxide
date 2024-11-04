pub mod arena;
mod bullet;
mod bullet_link;
pub mod collision_mesh_file;
pub mod consts;
mod custom;
mod framework;
pub mod math;
pub mod mutator_config;

use custom::bvh::Bvh;
use framework::RS_VERSION;
use std::{
    collections::{hash_map::RandomState, HashMap, HashSet},
    fs, io,
    path::{Path, PathBuf},
    sync::RwLock,
    time::SystemTime,
};

use crate::collision_mesh_file::{CollisionMeshFile, COLLISION_MESH_FILE_EXTENSION};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Stage {
    Uninitialized,
    Initializing,
    Initialized,
}

pub(crate) type BvhTriangleMeshShape = Bvh;

const SOCCAR_ARENA_MESH_HASHES: [u32; 16] = [
    0xA160BAF9, 0x2811EEE8, 0xB81AC8B9, 0x760358D3, 0x73AE4940, 0x918F4A4E, 0x1F8EE550, 0x255BA8C1,
    0x14B84668, 0xEC759EBF, 0x94FB0D5C, 0xDEA07102, 0xBD4FBEA8, 0x39A47F63, 0x3D79D25D, 0xD84C7A68,
];

static STAGE: RwLock<Stage> = RwLock::new(Stage::Uninitialized);

pub fn get_stage() -> Stage {
    *STAGE.read().unwrap()
}

pub(crate) static ARENA_COLLISION_MESH: RwLock<BvhTriangleMeshShape> =
    RwLock::new(BvhTriangleMeshShape::new());

#[inline]
pub fn init<P: AsRef<Path>>(collision_meshes_folder: P) -> io::Result<()> {
    init_from_path(collision_meshes_folder.as_ref())
}

fn init_from_path(collision_meshes_folder: &Path) -> io::Result<()> {
    let mut arena_collision_meshes = ARENA_COLLISION_MESH.write().unwrap();

    if get_stage() != Stage::Uninitialized {
        println!("WARNING: RocketSim::Init() called again after already initialized, ignoring...");
        return Ok(());
    }

    println!("Initializing RocketSim-rs version {RS_VERSION}, created by ZealanL & VirxEC...");

    *STAGE.write().unwrap() = Stage::Initializing;

    let start_time = SystemTime::now();

    load_soccer_meshes(
        collision_meshes_folder,
        collision_meshes_folder.join("soccar"),
        &mut arena_collision_meshes,
    )?;

    let elapsed_time = SystemTime::now().duration_since(start_time).unwrap();
    println!(
        "Finished initializing RocketSim in {}s!",
        elapsed_time.as_secs_f32()
    );

    *STAGE.write().unwrap() = Stage::Initialized;

    Ok(())
}

fn load_soccer_meshes(
    base_path: &Path,
    soccer_meshes_folder: PathBuf,
    arena_collision_meshes: &mut BvhTriangleMeshShape,
) -> io::Result<()> {
    const MSG_PREFIX: &str = "RocketSim::Init(): ";

    println!("Loading arena meshes from {soccer_meshes_folder:?}...");
    assert!(
        soccer_meshes_folder.exists(),
        "Failed to find arena collision mesh files at {soccer_meshes_folder:?}, the collision meshes folder should be in our current directory {:?}.",
        std::env::current_dir().unwrap()
    );

    // How many of each collision mesh hash we have loaded
    // There should be 1 for each of the SOCCAR_ARENA_MESH_HASHES

    let mut hash_counts = HashMap::new();
    let target_hashes = HashSet::<_, RandomState>::from_iter(SOCCAR_ARENA_MESH_HASHES);

    let mut num_meshes = 0;
    let mut primitives = Vec::with_capacity(SOCCAR_ARENA_MESH_HASHES.len());

    for entry in fs::read_dir(soccer_meshes_folder)?.flatten() {
        let entry_path = entry.path();

        let Some(ext) = entry_path.extension() else {
            continue;
        };

        if ext != COLLISION_MESH_FILE_EXTENSION {
            continue;
        }

        let mesh_file = CollisionMeshFile::read_from_file(&entry_path)?;
        let mesh_hash = mesh_file.get_hash();
        let hash_count = hash_counts.entry(mesh_hash).or_insert(0);

        if *hash_count > 0 {
            println!("WARNING: {MSG_PREFIX}Collision mesh {entry_path:?} is a duplicate (0x{mesh_hash:x}), already loaded a mesh with the same hash.");
        } else if !target_hashes.contains(&mesh_file.get_hash()) {
            println!("WARNING: {MSG_PREFIX}Collision mesh {entry_path:?} does not match any known soccar collision file (0x{mesh_hash:x}), make sure they were dumped from a normal soccar arena.");
        }
        *hash_count += 1;

        num_meshes += 1;
        mesh_file.add_triangles_to(&mut primitives);
    }

    assert!(
        num_meshes != 0,
        "{MSG_PREFIX}Failed to find soccar field asset files at {base_path:?}, the folder exists but has no collision mesh files."
    );

    CollisionMeshFile::add_extra_soccer_tris_to(&mut primitives);
    *arena_collision_meshes = Bvh::from(&primitives);

    println!(
        "{MSG_PREFIX}Finished loading {num_meshes} arena collision meshes ({} triangles).",
        arena_collision_meshes.num_leaves
    );

    Ok(())
}

pub(crate) fn assert_initialized(error_msg_prefix: &str) {
    assert!(
        get_stage() == Stage::Initialized,
        "{error_msg_prefix}RocketSim has not been initialized, call RocketSim::Init() first."
    );
}
