use super::{
    broadphase_interface::BroadphaseInterface,
    broadphase_proxy::{BroadphaseNativeTypes, BroadphaseProxy},
    overlapping_pair_cache::OverlappingPairCache,
};
use crate::{
    collision::{
        dispatch::collision_object::CollisionObject,
        shapes::{collision_shape::CollisionShapes, triangle_callback::TriangleCallback},
    },
    linear_math::aabb_util_2::test_aabb_against_aabb,
};
use glam::{USizeVec3, Vec3A};
use std::{cell::RefCell, rc::Rc};

#[derive(Clone, Default)]
pub struct RsBroadphaseProxy {
    pub(crate) broadphase_proxy: BroadphaseProxy,
    pub(crate) is_static: bool,
    pub(crate) cell_idx: usize,
    pub(crate) indices: USizeVec3,
    pub(crate) shape_type: BroadphaseNativeTypes,
    pub(crate) next_free: usize,
}

#[derive(Clone)]
struct Cell {
    dyn_handles: Vec<Rc<RefCell<RsBroadphaseProxy>>>,
    static_handles: Vec<Rc<RefCell<RsBroadphaseProxy>>>,
}

impl Default for Cell {
    fn default() -> Self {
        Self {
            dyn_handles: Vec::with_capacity(Self::RESERVED_SIZE),
            static_handles: Vec::with_capacity(Self::RESERVED_SIZE),
        }
    }
}

impl Cell {
    const RESERVED_SIZE: usize = 4;

    fn remove_static(&mut self, proxy: &Rc<RefCell<RsBroadphaseProxy>>) {
        if let Some(pos) = self
            .static_handles
            .iter()
            .position(|x| Rc::ptr_eq(x, proxy))
        {
            self.static_handles.remove(pos);
        }
    }

    fn remove_dyn(&mut self, proxy: &Rc<RefCell<RsBroadphaseProxy>>) {
        if let Some(pos) = self.dyn_handles.iter().position(|x| Rc::ptr_eq(x, proxy)) {
            self.dyn_handles.remove(pos);
        }
    }
}

#[derive(Default)]
struct BoolHitTriangleCallback {
    hit: bool,
}

impl TriangleCallback for BoolHitTriangleCallback {
    fn process_triangle(
        &mut self,
        _triangle: &[Vec3A],
        _tri_aabb_min: Vec3A,
        _tri_aabb_max: Vec3A,
        _part_id: usize,
        _triangle_index: usize,
    ) -> bool {
        self.hit = true;

        false
    }
}

pub struct RsBroadphase {
    num_handles: usize,
    max_handles: usize,
    last_handle_index: usize,
    max_pos: Vec3A,
    min_pos: Vec3A,
    cell_size: f32,
    cell_size_sq: f32,
    num_cells: USizeVec3,
    total_cells: usize,
    num_dyn_proxies: u32,
    total_static_pairs: u32,
    total_dyn_pairs: u32,
    total_real_pairs: u32,
    total_iters: u32,
    cells: Vec<Cell>,
    handles: Vec<Rc<RefCell<RsBroadphaseProxy>>>,
    first_free_handle: usize,
    pair_cache: Box<dyn OverlappingPairCache>,
    // bool m_ownsPairCache;
    // int m_invalidPair;
}

impl RsBroadphase {
    #[must_use]
    pub fn new(
        min_pos: Vec3A,
        max_pos: Vec3A,
        cell_size: f32,
        pair_cache: Box<dyn OverlappingPairCache>,
        max_handles: usize,
    ) -> Self {
        debug_assert!(min_pos.cmple(max_pos).all(), "Invalid min/max pos");

        let handles: Vec<_> = (0..max_handles)
            .map(|i| {
                Rc::new(RefCell::new(RsBroadphaseProxy {
                    broadphase_proxy: BroadphaseProxy {
                        unique_id: i as u32 + 2,
                        ..Default::default()
                    },
                    next_free: i + 1,
                    ..Default::default()
                }))
            })
            .collect();

        handles.last().unwrap().borrow_mut().next_free = 0;

        let range = max_pos - min_pos;
        let num_cells = (range / cell_size)
            .ceil()
            .as_usizevec3()
            .max(USizeVec3::ONE);
        let total_cells = num_cells.element_product();

        let cells = vec![Cell::default(); total_cells];

        Self {
            num_handles: 0,
            max_handles,
            last_handle_index: 0,
            max_pos,
            min_pos,
            cell_size,
            cell_size_sq: cell_size * cell_size,
            num_cells,
            total_cells,
            num_dyn_proxies: 0,
            total_static_pairs: 0,
            total_dyn_pairs: 0,
            total_real_pairs: 0,
            total_iters: 0,
            cells,
            handles,
            first_free_handle: 0,
            pair_cache,
        }
    }

    fn alloc_handle(&mut self) -> usize {
        debug_assert!(self.num_handles < self.max_handles);

        let free_handle = self.first_free_handle;
        self.first_free_handle = self.handles[free_handle].borrow().next_free;
        self.num_handles += 1;

        if free_handle > self.last_handle_index {
            self.last_handle_index = free_handle;
        }

        free_handle
    }

    fn get_cell_indices(&self, pos: Vec3A) -> USizeVec3 {
        let cell_idx_f = (pos - self.min_pos) / self.cell_size;
        cell_idx_f
            .as_usizevec3()
            .clamp(USizeVec3::ZERO, self.num_cells - USizeVec3::ONE)
    }

    fn get_cell_index(&self, pos: Vec3A) -> usize {
        self.cell_indices_to_index(self.get_cell_indices(pos))
    }

    const fn cell_indices_to_index(&self, indices: USizeVec3) -> usize {
        indices.x * self.num_cells.y * self.num_cells.z + indices.y * self.num_cells.z + indices.z
    }

    fn get_cell_min_pos(&self, indices: USizeVec3) -> Vec3A {
        self.min_pos + indices.as_vec3a() * self.cell_size
    }

    fn get_cell(&mut self, indices: USizeVec3) -> &mut Cell {
        let idx = self.cell_indices_to_index(indices);
        &mut self.cells[idx]
    }

    fn update_cells_static<const ADD: bool>(&mut self, proxy: &Rc<RefCell<RsBroadphaseProxy>>) {
        let proxy_ref = proxy.borrow();

        // TODO: "Fix dumb massive value aabb bug"
        let aabb_max = proxy_ref.broadphase_proxy.aabb_max.min(self.max_pos);

        let min = self.get_cell_indices(proxy_ref.broadphase_proxy.aabb_min);
        let max = self.get_cell_indices(aabb_max);

        let col_obj = &*proxy_ref
            .broadphase_proxy
            .client_object
            .as_ref()
            .unwrap()
            .borrow();
        let obj = col_obj.get_collision_shape().as_ref().unwrap().borrow();
        let tri_mesh_shape = match &*obj {
            CollisionShapes::TriangleMesh(mesh) => Some(mesh),
            _ => None,
        };

        let mut callback_inst = BoolHitTriangleCallback::default();
        let mut cells = Vec::with_capacity(27);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    debug_assert!(cells.is_empty());
                    let idx = i * self.num_cells.y * self.num_cells.z + j * self.num_cells.z + k;

                    if ADD {
                        if let Some(mesh_interface) = tri_mesh_shape {
                            let cell_min = self.get_cell_min_pos(USizeVec3::new(i, j, k));
                            let cell_max = cell_min + Vec3A::splat(self.cell_size);

                            callback_inst.hit = false;
                            mesh_interface.process_all_triangles(
                                &mut callback_inst,
                                cell_min,
                                cell_max,
                            );

                            if idx == 2862 {
                                dbg!(idx);
                            }

                            if !callback_inst.hit {
                                continue;
                            }
                        }
                    }

                    for i1 in 0..=2 {
                        for j1 in 0..=2 {
                            for k1 in 0..=2 {
                                let mut cell = USizeVec3::new(i + i1, j + j1, k + k1);
                                if cell.cmpeq(USizeVec3::ZERO).any() {
                                    continue;
                                }

                                cell -= USizeVec3::ONE;

                                if cell.cmpge(self.num_cells).any() {
                                    continue;
                                }

                                cells.push(self.cell_indices_to_index(cell));
                            }
                        }
                    }

                    for &i in &cells {
                        if ADD {
                            let mut already_exists = false;
                            for static_handle in &self.cells[i].static_handles {
                                // check if static_handle and proxy are the same
                                if Rc::ptr_eq(static_handle, proxy) {
                                    already_exists = true;
                                    break;
                                }
                            }

                            if !already_exists {
                                self.cells[i].static_handles.push(proxy.clone());
                            }
                        } else {
                            self.cells[i].remove_static(proxy);
                        }
                    }

                    cells.clear();
                }
            }
        }
    }

    fn update_cells_dynamic<const ADD: bool>(
        &mut self,
        proxy: &Rc<RefCell<RsBroadphaseProxy>>,
        indices: USizeVec3,
    ) {
        let min = USizeVec3::ONE.max(indices) - USizeVec3::ONE;
        let max = (self.num_cells - USizeVec3::ONE).min(indices + USizeVec3::ONE);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    let idx = i * self.num_cells.y * self.num_cells.z + j * self.num_cells.z + k;
                    let cell = self.get_cell(USizeVec3::new(i, j, k));

                    if ADD {
                        if idx == 2862 {
                            continue;
                        }
                        cell.dyn_handles.push(proxy.clone());
                    } else {
                        cell.remove_dyn(proxy);
                    }
                }
            }
        }
    }

    fn aabb_overlap(proxy0: &RsBroadphaseProxy, proxy1: &RsBroadphaseProxy) -> bool {
        test_aabb_against_aabb(
            proxy0.broadphase_proxy.aabb_min,
            proxy0.broadphase_proxy.aabb_max,
            proxy1.broadphase_proxy.aabb_min,
            proxy1.broadphase_proxy.aabb_max,
        )
    }
}

impl BroadphaseInterface for RsBroadphase {
    fn set_aabb(
        &mut self,
        proxy: &Rc<RefCell<RsBroadphaseProxy>>,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
    ) {
        let sbp = proxy.borrow();

        if sbp.broadphase_proxy.aabb_min != aabb_min || sbp.broadphase_proxy.aabb_max != aabb_max {
            if sbp.is_static {
                drop(sbp);
                self.update_cells_static::<false>(proxy);

                let mut sbp = proxy.borrow_mut();
                sbp.broadphase_proxy.aabb_min = aabb_min;
                sbp.broadphase_proxy.aabb_max = aabb_max;
                drop(sbp);

                self.update_cells_static::<true>(proxy);
            } else {
                drop(sbp);
                let mut sbp = proxy.borrow_mut();
                let old_index = sbp.cell_idx;
                sbp.broadphase_proxy.aabb_min = aabb_min;
                sbp.broadphase_proxy.aabb_max = aabb_max;

                let new_indices = self.get_cell_indices(aabb_min);
                let new_index = self.cell_indices_to_index(new_indices);
                sbp.cell_idx = new_index;

                if new_index != old_index && self.num_dyn_proxies > 1 {
                    drop(sbp);
                    self.update_cells_dynamic::<false>(proxy, new_indices);

                    let mut sbp = proxy.borrow_mut();
                    sbp.indices = new_indices;
                    drop(sbp);

                    self.update_cells_dynamic::<true>(proxy, new_indices);
                }
            }
        }
    }

    fn create_proxy(
        &mut self,
        aabb_min: Vec3A,
        aabb_max: Vec3A,
        shape_type: BroadphaseNativeTypes,
        // user_ptr: *mut std::ffi::c_void,
        collision_object: Rc<RefCell<CollisionObject>>,
        collision_filter_group: i32,
        collision_filter_mask: i32,
        // dispatcher: &CollisionDispatcher,
    ) -> Rc<RefCell<RsBroadphaseProxy>> {
        debug_assert!(aabb_min.cmple(aabb_max).all());

        let is_static = collision_object.borrow().is_static_object();
        let new_handle_idx = self.alloc_handle();
        let cell_idx = self.get_cell_index(aabb_min);
        let indices = self.get_cell_indices(aabb_min);

        let new_handle = RsBroadphaseProxy {
            broadphase_proxy: BroadphaseProxy {
                aabb_min,
                aabb_max,
                client_object: Some(collision_object),
                collision_filter_group,
                collision_filter_mask,
                unique_id: self.handles[new_handle_idx]
                    .borrow()
                    .broadphase_proxy
                    .unique_id,
            },
            is_static,
            cell_idx,
            indices,
            shape_type,
            next_free: 0,
        };

        *self.handles[new_handle_idx].borrow_mut() = new_handle;
        let proxy = self.handles[new_handle_idx].clone();

        if is_static {
            self.update_cells_static::<true>(&proxy);
        } else {
            debug_assert!(aabb_min.distance_squared(aabb_max) <= self.cell_size_sq);

            self.update_cells_dynamic::<true>(&proxy, indices);
            self.num_dyn_proxies += 1;
        }

        self.handles[new_handle_idx].clone()
    }

    fn calculate_overlapping_pairs(&mut self) {
        // let last_real_pairs = self.total_real_pairs;

        debug_assert!(self.pair_cache.is_empty());
        if self.num_handles == 0 {
            return;
        }

        let mut new_largest_index = 0;
        for (i, proxy) in self.handles[..=self.last_handle_index].iter().enumerate() {
            let proxy_ref = proxy.borrow();
            if proxy_ref.is_static {
                continue; // todo: use separate list
            }

            self.total_iters += 1;
            new_largest_index = i;

            let cell = &self.cells[proxy_ref.cell_idx];

            for other_proxy in &cell.static_handles {
                let other_proxy_ref = other_proxy.borrow();

                self.total_static_pairs += 1;

                if Self::aabb_overlap(&proxy_ref, &other_proxy_ref)
                    && !self.pair_cache.contains_pair(&proxy_ref, &other_proxy_ref)
                {
                    self.pair_cache.add_overlapping_pair(proxy, other_proxy);
                    self.total_real_pairs += 1;
                }
            }

            if self.num_dyn_proxies > 1 {
                todo!()
            }
        }

        self.last_handle_index = new_largest_index;
    }

    fn get_overlapping_pair_cache(&mut self) -> &mut dyn OverlappingPairCache {
        self.pair_cache.as_mut()
    }
}
