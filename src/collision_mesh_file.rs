use std::{
    fs,
    io::{self, Cursor},
    num::Wrapping,
    path::Path,
};

use byteorder::{LittleEndian, ReadBytesExt};

use crate::{
    bullet::linear_math::vector3::Vector3,
    bullet_link::UU_TO_BT,
    consts::{ARENA_EXTENT_X, ARENA_EXTENT_Y, ARENA_HEIGHT},
    custom::geometry::Tri,
};

pub const COLLISION_MESH_BASE_PATH: &str = "./collision_meshes/";
pub const COLLISION_MESH_FILE_EXTENSION: &str = "cmf";

trait FromCursor {
    fn from_cursor(bytes: &mut Cursor<Vec<u8>>) -> io::Result<Self>
    where
        Self: Sized;
}

impl FromCursor for Vector3 {
    fn from_cursor(bytes: &mut Cursor<Vec<u8>>) -> io::Result<Self> {
        Ok(Self::new(
            bytes.read_f32::<LittleEndian>()?,
            bytes.read_f32::<LittleEndian>()?,
            bytes.read_f32::<LittleEndian>()?,
        ))
    }
}

pub(crate) struct CollisionMeshFile {
    indices: Vec<usize>,
    vertices: Vec<Vector3>,
    hash: u32,
}

impl CollisionMeshFile {
    #[inline]
    fn quad(p: Vector3, e1: Vector3, e2: Vector3) -> Self {
        Self {
            indices: vec![0, 1, 3, 1, 2, 3],
            vertices: vec![p + e1 + e2, p - e1 + e2, p - e1 - e2, p + e1 - e2],
            hash: 0,
        }
    }

    pub fn add_extra_soccer_tris_to(tris: &mut Vec<Tri>) {
        let arena_extent = Vector3::new(ARENA_EXTENT_X, ARENA_EXTENT_Y, ARENA_HEIGHT) * UU_TO_BT;

        let planes = [
            // floor
            Self::quad(
                Vector3::ZERO,
                Vector3::new(arena_extent.x, 0., 0.),
                Vector3::new(0., 5500. * UU_TO_BT, 0.),
            ),
            // ceiling
            Self::quad(
                Vector3::new(0., 0., arena_extent.z),
                Vector3::new(-arena_extent.x, 0., 0.),
                Vector3::new(0., arena_extent.y, 0.),
            ),
            // left wall
            Self::quad(
                Vector3::new(-arena_extent.x, 0., arena_extent.z / 2.),
                Vector3::new(0., arena_extent.y, 0.),
                Vector3::new(0., 0., arena_extent.z / 2.),
            ),
            // right wall
            Self::quad(
                Vector3::new(arena_extent.x, 0., arena_extent.z / 2.),
                Vector3::new(0., -arena_extent.y, 0.),
                Vector3::new(0., 0., arena_extent.z / 2.),
            ),
        ];

        for plane in planes {
            plane.add_triangles_to(tris);
        }
    }

    #[inline]
    pub fn get_hash(&self) -> u32 {
        self.hash
    }

    /// From: https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/72073933#72073933
    fn calculate_hash(indices: &Vec<usize>, vertices: &[Vector3]) -> u32 {
        let mut hash = Wrapping((vertices.len() + (indices.len() / 3 * vertices.len())) as u32);

        const HASH_VAL_MUELLER: Wrapping<u32> = Wrapping(0x45D9F3B);
        const HASH_VAL_SHIFT: Wrapping<u32> = Wrapping(0x9E3779B9);

        for &vert_index in indices {
            for pos in vertices[vert_index].to_array() {
                let mut cur_val = Wrapping(pos as i32 as u32);
                cur_val = ((cur_val >> 16) ^ cur_val) * HASH_VAL_MUELLER;
                cur_val = ((cur_val >> 16) ^ cur_val) * HASH_VAL_MUELLER;
                cur_val = (cur_val >> 16) ^ cur_val;
                hash ^= cur_val + HASH_VAL_SHIFT + (hash << 6) + (hash >> 2);
            }
        }

        hash.0
    }

    pub fn read_from_file(file_path: &Path) -> io::Result<Self> {
        const ERROR_PREFIX_STR: &str = " > CollisionMeshFile::ReadFromFile(): ";
        let mut bytes = Cursor::new(fs::read(file_path)?);

        const MAX_VERT_OR_TRI_COUNT: usize = 1000 * 1000;

        let num_tris = bytes.read_i32::<LittleEndian>()? as usize;
        let num_indices = num_tris * 3;
        let num_vertices = bytes.read_i32::<LittleEndian>()? as usize;

        assert!(num_tris.min(num_vertices) != 0 && num_tris.max(num_vertices) <= MAX_VERT_OR_TRI_COUNT, "{ERROR_PREFIX_STR}Invalid collision mesh file at \"{file_path:?}\" (bad triangle/vertex count: [{num_tris}, {num_vertices}])");

        let mut indices = Vec::with_capacity(num_indices);
        indices.extend(
            (0..num_indices).flat_map(|_| bytes.read_i32::<LittleEndian>().map(|i| i as usize)),
        );
        debug_assert!(indices.len() == num_indices as usize);

        let mut vertices = Vec::with_capacity(num_vertices as usize);
        vertices.extend((0..num_vertices).flat_map(|_| Vector3::from_cursor(&mut bytes)));
        debug_assert!(vertices.len() == num_vertices as usize);

        if cfg!(debug_assertions) {
            // Verify that the triangle data is correct
            indices.iter().for_each(|&vert_index| {
                assert!(
                    vert_index < num_vertices,
                    "{ERROR_PREFIX_STR}Invalid collision mesh file at \"{file_path:?}\" (bad triangle vertex index)"
                );
            });
        }

        let hash = Self::calculate_hash(&indices, &vertices);

        println!("   > Loaded {num_vertices} verts and {num_tris} tris, hash: 0x{hash:x}");

        Ok(Self {
            indices,
            vertices,
            hash,
        })
    }

    pub fn add_triangles_to(self, tris: &mut Vec<Tri>) {
        tris.reserve(self.indices.len() / 3);
        tris.extend(self.indices.chunks_exact(3).map(|chunk| {
            Tri::from_points(
                self.vertices[chunk[0]],
                self.vertices[chunk[1]],
                self.vertices[chunk[2]],
            )
        }));
    }
}
