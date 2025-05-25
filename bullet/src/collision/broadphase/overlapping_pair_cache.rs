use super::{broadphase_proxy::BroadphasePair, overlapping_pair_callback::OverlappingPairCallback};

pub trait OverlapCallback {
    fn process_overlap(&mut self, pair: &BroadphasePair);
}

pub trait OverlapFilterCallback {
    fn needs_broadphase_collision(&self, proxy0: &BroadphasePair, proxy1: &BroadphasePair) -> bool;
}

pub trait OverlappingPairCache: OverlappingPairCallback {}

#[derive(Default)]
pub struct HashedOverlappingPairCache {
    pub overlapping_pair_array: Vec<BroadphasePair>,
    pub overlap_filter_callback: Option<Box<dyn OverlapFilterCallback>>,
    hash_table: Vec<i32>,
    next: Vec<i32>,
    ghost_pair_callback: Option<Box<dyn OverlappingPairCallback>>,
}

impl OverlappingPairCallback for HashedOverlappingPairCache {}
impl OverlappingPairCache for HashedOverlappingPairCache {}
