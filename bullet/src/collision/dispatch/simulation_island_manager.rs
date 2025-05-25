use super::union_find::UnionFind;

pub trait IslandCallback {}

pub struct SimulationIslandManager {
    union_find: UnionFind,
    // btAlignedObjectArray<btPersistentManifold*> m_islandmanifold;
    // btAlignedObjectArray<btCollisionObject*> m_islandBodies;
    split_islands: bool,
}

impl Default for SimulationIslandManager {
    fn default() -> Self {
        Self {
            union_find: UnionFind::default(),
            split_islands: true,
        }
    }
}
