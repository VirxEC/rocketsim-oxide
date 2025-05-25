use super::manifold_point::ManifoldPoint;

const MANIFOLD_CACHE_SIZE: usize = 4;

pub struct PersistentManifold {
    object_type: i32,
    point_cache: [ManifoldPoint; MANIFOLD_CACHE_SIZE],
    // const btCollisionObject* m_body0;
    // const btCollisionObject* m_body1;
    cached_points: i32,
    contact_breaking_threshold: f32,
    contact_processing_threshold: f32,
    pub companion_id_a: i32,
    pub companion_id_b: i32,
    index_1a: i32,
}
