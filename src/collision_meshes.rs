use crate::bullet::collision::shapes::triangle_mesh::TriangleMesh;
use byteorder::{LittleEndian, ReadBytesExt};
use glam::Vec3A;
use std::{
    io::{Cursor, Result as IoResult},
    num::Wrapping,
};

pub const COLLISION_MESH_BASE_PATH: &str = "./collision_meshes/";
pub const COLLISION_MESH_FILE_EXTENSION: &str = "cmf";

trait FromCursor {
    fn from_cursor(bytes: &mut Cursor<&[u8]>) -> IoResult<Self>
    where
        Self: Sized;
}

impl FromCursor for Vec3A {
    fn from_cursor(bytes: &mut Cursor<&[u8]>) -> IoResult<Self> {
        Ok(Self::new(
            bytes.read_f32::<LittleEndian>()?,
            bytes.read_f32::<LittleEndian>()?,
            bytes.read_f32::<LittleEndian>()?,
        ))
    }
}

pub struct CollisionMeshFile {
    indices: Vec<usize>,
    vertices: Vec<Vec3A>,
    hash: u32,
}

impl CollisionMeshFile {
    #[inline]
    pub const fn get_hash(&self) -> u32 {
        self.hash
    }

    /// From: <https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/72073933#72073933>
    fn calculate_hash(indices: &Vec<usize>, vertices: &[Vec3A]) -> u32 {
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

    pub fn read_from_bytes(bytes: &[u8], silent: bool) -> IoResult<Self> {
        const ERROR_PREFIX_STR: &str = " > CollisionMeshFile::ReadFromFile(): ";
        const MAX_VERT_OR_TRI_COUNT: usize = 1000 * 1000;

        let mut bytes = Cursor::new(bytes);
        let num_tris = bytes.read_u32::<LittleEndian>()? as usize;
        let num_indices = num_tris * 3;
        let num_vertices = bytes.read_u32::<LittleEndian>()? as usize;

        assert!(
            num_tris.min(num_vertices) != 0 && num_tris.max(num_vertices) <= MAX_VERT_OR_TRI_COUNT,
            "{ERROR_PREFIX_STR}Invalid collision mesh file (bad triangle/vertex count: [{num_tris}, {num_vertices}])"
        );

        let indices = (0..num_indices)
            .map(|_| bytes.read_u32::<LittleEndian>().map(|x| x as usize))
            .collect::<Result<Vec<_>, _>>()?;
        let vertices = (0..num_vertices)
            .map(|_| Vec3A::from_cursor(&mut bytes))
            .collect::<Result<Vec<_>, _>>()?;

        #[cfg(debug_assertions)]
        {
            // Verify that the triangle data is correct
            for &vert_index in &indices {
                assert!(
                    vert_index < num_vertices,
                    "{ERROR_PREFIX_STR}Invalid collision mesh file (bad triangle vertex index)"
                );
            }
        }

        let hash = Self::calculate_hash(&indices, &vertices);

        if !silent {
            println!("   > Loaded {num_vertices} verts and {num_tris} tris, hash: {hash:#x}");
        }

        Ok(Self {
            indices,
            vertices,
            hash,
        })
    }

    pub fn make_bullet_mesh(self) -> TriangleMesh {
        TriangleMesh::new(self.vertices, self.indices)
    }
}
