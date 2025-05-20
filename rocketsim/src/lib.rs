pub use ahash;
pub use bullet;

pub mod consts;

mod collision_meshes;

use ahash::AHashMap;
use bullet::collision::{
    dispatch::internal_edge_utility::generate_internal_edge_info,
    shapes::{bvh_triangle_mesh_shape::BvhTriangleMeshShape, triangle_info_map::TriangleInfoMap},
};
use collision_meshes::{
    COLLISION_MESH_BASE_PATH, COLLISION_MESH_FILE_EXTENSION, CollisionMeshFile,
};
use std::{
    fs,
    io::{Error as IoError, ErrorKind, Result as IoResult},
    path::Path,
    time::Instant,
};

/// BulletPhysics Units (1m) to Unreal Units (2cm) conversion scale
pub(crate) const BT_TO_UU: f32 = 50.0;

/// Unreal Units (2cm) to BulletPhysics Units (1m) conversion scale
pub(crate) const UU_TO_BT: f32 = 1.0 / 50.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GameMode {
    Soccar,
    Hoops,
    Heatseeker,
    Snowday,
    Dropshot,
    /// Soccar but without goals, boost pads, or the arena hull. The cars and ball will fall infinitely.
    TheVoid,
}

impl GameMode {
    const NAMES: [&'static str; 7] = [
        "soccar",
        "hoops",
        "heatseeker",
        "snowday",
        "dropshot",
        "dropshot",
        "void",
    ];

    #[must_use]
    pub const fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub(crate) fn get_hashes(self) -> AHashMap<u32, u32> {
        macro_rules! zero_iter {
            ($($i:literal),+) => {
                [
                    $(($i, 0)),+
                ].into_iter()
            }
        }

        match self {
            Self::Soccar => zero_iter![
                0xA160BAF9, 0x2811EEE8, 0xB81AC8B9, 0x760358D3, 0x73AE4940, 0x918F4A4E, 0x1F8EE550,
                0x255BA8C1, 0x14B84668, 0xEC759EBF, 0x94FB0D5C, 0xDEA07102, 0xBD4FBEA8, 0x39A47F63,
                0x3D79D25D, 0xD84C7A68
            ]
            .collect(),
            Self::Hoops => zero_iter![
                0x72F2359E, 0x5ED14A26, 0xFD5A0D07, 0x92AFA5B5, 0x0E4133C7, 0x399E8B5F, 0xBB9D4FB5,
                0x8C87FB93, 0x1CFD0E16, 0xE19E1DF6, 0x9CA179DC, 0x16F3CC19
            ]
            .collect(),
            Self::Dropshot => zero_iter![0x7eb0b2d3, 0x911041d2].collect(),
            _ => AHashMap::new(),
        }
    }
}

pub fn init_from_default(silent: bool) -> IoResult<()> {
    init(COLLISION_MESH_BASE_PATH, silent)
}

pub fn init<P: AsRef<Path>>(collision_meshes_folder: P, silent: bool) -> IoResult<()> {
    init_from_path(collision_meshes_folder.as_ref(), silent)
}

fn init_from_path(collision_meshes_folder: &Path, silent: bool) -> IoResult<()> {
    const GAMEMODES_WITH_UNIQUE_MESHES: [GameMode; 3] =
        [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot];

    if !collision_meshes_folder.exists() {
        return IoResult::Err(IoError::new(
            ErrorKind::NotFound,
            format!("{} does not exist", collision_meshes_folder.display()),
        ));
    }

    if !collision_meshes_folder.is_dir() {
        return IoResult::Err(IoError::new(
            ErrorKind::NotADirectory,
            format!("{} is not a directory", collision_meshes_folder.display()),
        ));
    }

    let mut mesh_file_map = AHashMap::new();

    for game_mode in GAMEMODES_WITH_UNIQUE_MESHES {
        let folder = collision_meshes_folder.join(game_mode.name());
        if !folder.exists() {
            continue;
        }

        let mut files = Vec::new();

        for entry in fs::read_dir(folder)?.flatten() {
            let p = entry.path();

            if !p.is_file()
                || p.extension()
                    .is_none_or(|ext| ext != COLLISION_MESH_FILE_EXTENSION)
            {
                continue;
            }

            files.push(fs::read(p)?);
        }

        mesh_file_map.insert(game_mode, files);
    }

    init_from_mem(mesh_file_map, silent)?;

    Ok(())
}

pub fn init_from_mem(
    mesh_file_map: AHashMap<GameMode, Vec<Vec<u8>>>,
    silent: bool,
) -> IoResult<AHashMap<GameMode, Vec<BvhTriangleMeshShape>>> {
    if !silent {
        println!("Initializing RocketSim, originally by Zealan and ported to Rust by Virx...");
    }

    let start_time = Instant::now();

    // DropshotTiles::Init();

    let mut arena_collision_shapes = AHashMap::new();

    for (game_mode, mesh_files) in mesh_file_map {
        if !silent {
            println!("Loading arena meshes for {}...", game_mode.name());
        }

        if mesh_files.is_empty() {
            if !silent {
                println!("No meshes, skipping");
            }
            continue;
        }

        let mut meshes = Vec::with_capacity(mesh_files.len());
        let mut target_hashes = game_mode.get_hashes();

        for (i, entry) in mesh_files.into_iter().enumerate() {
            let mesh_file = CollisionMeshFile::read_from_bytes(&entry, silent)?;
            let hash = mesh_file.get_hash();
            let Some(hash_count) = target_hashes.get_mut(&hash) else {
                eprintln!(
                    "Collision mesh [{i}] does not match any known {} collision mesh ({hash:#x}), make sure they were dumped form a normal {} arena.",
                    game_mode.name(),
                    game_mode.name()
                );
                continue;
            };

            if *hash_count > 0 && !silent {
                eprintln!(
                    "Collision mesh [{i}] is a duplicate ({hash:#x}), already loaded a mesh with the same hash."
                );
            }

            *hash_count += 1;

            let tri_mesh = mesh_file.make_bullet_mesh();
            let mut bvt_mesh = BvhTriangleMeshShape::new(tri_mesh.into_mesh_interface(), true);
            let mut info_map = TriangleInfoMap::default();
            generate_internal_edge_info(&mut bvt_mesh, &mut info_map);
            bvt_mesh.set_triangle_info_map(info_map);
            meshes.push(bvt_mesh);
        }

        arena_collision_shapes.insert(game_mode, meshes);
    }

    if !silent {
        println!("Finished loading arena collision meshes:");
        println!(
            " > Soccar: {}",
            arena_collision_shapes
                .get(&GameMode::Soccar)
                .map_or(0, Vec::len)
        );
        println!(
            " > Hoops: {}",
            arena_collision_shapes
                .get(&GameMode::Hoops)
                .map_or(0, Vec::len)
        );
        println!(
            " > Dropshot: {}",
            arena_collision_shapes
                .get(&GameMode::Dropshot)
                .map_or(0, Vec::len)
        );

        let elapsed_time = Instant::now().duration_since(start_time);
        println!(
            "Finished initializing RocketSim in {:.3}s!",
            elapsed_time.as_secs_f32()
        );
    }

    Ok(arena_collision_shapes)
}
