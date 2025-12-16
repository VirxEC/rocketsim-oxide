use std::mem;

use ahash::AHashMap;

use super::{broadphase_proxy::BroadphasePair, rs_broadphase::RsBroadphaseProxy};
use crate::bullet::collision::{
    dispatch::collision_dispatcher::CollisionDispatcher,
    narrowphase::persistent_manifold::ContactAddedCallback,
};

pub struct HashedOverlappingPairCache {
    overlapping_pair_array: Vec<BroadphasePair>,
    hash_table: AHashMap<(usize, usize), usize>,
}

impl Default for HashedOverlappingPairCache {
    fn default() -> Self {
        let overlapping_pair_array = Vec::with_capacity(2);
        let new_capacity = overlapping_pair_array.capacity();

        Self {
            overlapping_pair_array,
            hash_table: AHashMap::with_capacity(new_capacity),
        }
    }
}

impl HashedOverlappingPairCache {
    fn internal_add_pair(
        &mut self,
        mut proxy0_id: usize,
        mut proxy0_idx: usize,
        mut proxy1_id: usize,
        mut proxy1_idx: usize,
    ) {
        if proxy0_id > proxy1_id {
            mem::swap(&mut proxy0_id, &mut proxy1_id);
            mem::swap(&mut proxy0_idx, &mut proxy1_idx);
        }

        self.hash_table
            .insert((proxy0_id, proxy1_id), self.overlapping_pair_array.len());

        self.overlapping_pair_array.push(BroadphasePair {
            proxy0: proxy0_idx,
            proxy1: proxy1_idx,
        });
    }

    pub fn add_overlapping_pair(
        &mut self,
        proxy0: &RsBroadphaseProxy,
        proxy0_idx: usize,
        proxy1: &RsBroadphaseProxy,
        proxy1_idx: usize,
    ) {
        if !Self::needs_broadphase_collision(proxy0, proxy1) {
            return;
        }

        self.internal_add_pair(
            proxy0.broadphase_proxy.unique_id,
            proxy0_idx,
            proxy1.broadphase_proxy.unique_id,
            proxy1_idx,
        );
    }

    pub fn is_empty(&self) -> bool {
        self.hash_table.is_empty()
    }

    pub fn contains_pair<'a>(
        &self,
        mut proxy0: &'a RsBroadphaseProxy,
        mut proxy1: &'a RsBroadphaseProxy,
    ) -> bool {
        if proxy0.broadphase_proxy.unique_id > proxy1.broadphase_proxy.unique_id {
            mem::swap(&mut proxy0, &mut proxy1);
        }

        self.hash_table.contains_key(&(
            proxy0.broadphase_proxy.unique_id,
            proxy1.broadphase_proxy.unique_id,
        ))
    }

    pub const fn needs_broadphase_collision(
        proxy0: &RsBroadphaseProxy,
        proxy1: &RsBroadphaseProxy,
    ) -> bool {
        (proxy0.broadphase_proxy.collision_filter_group
            & proxy1.broadphase_proxy.collision_filter_mask)
            != 0
            && (proxy1.broadphase_proxy.collision_filter_group
                & proxy0.broadphase_proxy.collision_filter_mask)
                != 0
    }

    pub fn process_all_overlapping_pairs<T: ContactAddedCallback>(
        &mut self,
        dispatcher: &mut CollisionDispatcher,
        handles: &[RsBroadphaseProxy],
        contact_added_callback: &mut T,
    ) {
        for pair in &self.overlapping_pair_array {
            dispatcher.near_callback(
                &handles[pair.proxy0],
                &handles[pair.proxy1],
                contact_added_callback,
            );
        }

        self.overlapping_pair_array.clear();
        self.hash_table.clear();
    }
}
