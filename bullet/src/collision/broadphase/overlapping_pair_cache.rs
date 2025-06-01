use super::{
    broadphase_proxy::BroadphasePair, overlapping_pair_callback::OverlappingPairCallback,
    rs_broadphase::RsBroadphaseProxy,
};
use crate::collision::dispatch::collision_dispatcher::CollisionDispatcher;
use ahash::AHashMap;
use std::{cell::RefCell, mem, rc::Rc};

pub trait OverlapCallback {
    fn process_overlap(&mut self, pair: &BroadphasePair) -> bool;
}

pub trait OverlapFilterCallback {
    fn needs_broadphase_collision(&self, proxy0: &BroadphasePair, proxy1: &BroadphasePair) -> bool;
}

pub trait OverlappingPairCache: OverlappingPairCallback {
    fn is_empty(&self) -> bool;

    fn contains_pair(&mut self, proxy0: &RsBroadphaseProxy, proxy1: &RsBroadphaseProxy) -> bool;

    fn needs_broadphase_collision(
        &self,
        proxy0: &RsBroadphaseProxy,
        proxy1: &RsBroadphaseProxy,
    ) -> bool;

    fn process_all_overlapping_pairs(&mut self, dispatcher: &mut CollisionDispatcher);
}

pub struct HashedOverlappingPairCache {
    overlapping_pair_array: Vec<BroadphasePair>,
    overlap_filter_callback: Option<Box<dyn OverlapFilterCallback>>,
    hash_table: AHashMap<(u32, u32), usize>,
    ghost_pair_callback: Option<Box<dyn OverlappingPairCallback>>,
}

impl Default for HashedOverlappingPairCache {
    fn default() -> Self {
        let overlapping_pair_array = Vec::with_capacity(2);
        let new_capacity = overlapping_pair_array.capacity();

        Self {
            overlapping_pair_array,
            overlap_filter_callback: None,
            hash_table: AHashMap::with_capacity(new_capacity),
            ghost_pair_callback: None,
        }
    }
}

impl HashedOverlappingPairCache {
    fn internal_add_pair<'a>(
        &mut self,
        mut proxy0: &'a Rc<RefCell<RsBroadphaseProxy>>,
        mut proxy1: &'a Rc<RefCell<RsBroadphaseProxy>>,
    ) {
        if proxy0.borrow().broadphase_proxy.unique_id > proxy1.borrow().broadphase_proxy.unique_id {
            mem::swap(&mut proxy0, &mut proxy1);
        }

        let proxy_id_1 = proxy0.borrow().broadphase_proxy.unique_id;
        let proxy_id_2 = proxy1.borrow().broadphase_proxy.unique_id;

        if let Some(_callback) = self.ghost_pair_callback.as_ref() {
            todo!()
        }

        self.hash_table
            .insert((proxy_id_1, proxy_id_2), self.overlapping_pair_array.len());

        self.overlapping_pair_array.push(BroadphasePair {
            proxy0: proxy0.clone(),
            proxy1: proxy1.clone(),
        });
    }
}

impl OverlappingPairCallback for HashedOverlappingPairCache {
    fn add_overlapping_pair(
        &mut self,
        proxy0: &Rc<RefCell<RsBroadphaseProxy>>,
        proxy1: &Rc<RefCell<RsBroadphaseProxy>>,
    ) {
        if !self.needs_broadphase_collision(&proxy0.borrow(), &proxy1.borrow()) {
            return;
        }

        self.internal_add_pair(proxy0, proxy1);
    }
}

impl OverlappingPairCache for HashedOverlappingPairCache {
    fn is_empty(&self) -> bool {
        self.hash_table.is_empty()
    }

    fn contains_pair<'a>(
        &mut self,
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

    fn needs_broadphase_collision(
        &self,
        proxy0: &RsBroadphaseProxy,
        proxy1: &RsBroadphaseProxy,
    ) -> bool {
        if let Some(_callback) = self.overlap_filter_callback.as_ref() {
            todo!()
        }

        (proxy0.broadphase_proxy.collision_filter_group
            & proxy1.broadphase_proxy.collision_filter_mask)
            != 0
            && (proxy1.broadphase_proxy.collision_filter_group
                & proxy0.broadphase_proxy.collision_filter_mask)
                != 0
    }

    fn process_all_overlapping_pairs(&mut self, dispatcher: &mut CollisionDispatcher) {
        for pair in &self.overlapping_pair_array {
            dispatcher.near_callback(pair);
        }

        self.overlapping_pair_array.clear();
        self.hash_table.clear();
    }
}
