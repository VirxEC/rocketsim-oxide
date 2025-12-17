use std::{cell::RefCell, rc::Rc};

use glam::{USizeVec3, Vec3A};

use super::{
    broadphase_proxy::BroadphaseProxy, overlapping_pair_cache::HashedOverlappingPairCache,
};
use crate::bullet::{
    collision::{
        broadphase::broadphase_proxy::BroadphaseAabbCallback,
        dispatch::{collision_dispatcher::CollisionDispatcher, collision_object::CollisionObject},
        narrowphase::persistent_manifold::ContactAddedCallback,
        shapes::{
            collision_shape::CollisionShapes, triangle_callback::TriangleCallback,
            triangle_shape::TriangleShape,
        },
    },
    dynamics::rigid_body::RigidBody,
    linear_math::aabb_util_2::{Aabb, test_aabb_against_aabb},
};

#[derive(Clone, Default)]
pub struct RsBroadphaseProxy {
    pub(crate) broadphase_proxy: BroadphaseProxy,
    pub(crate) is_static: bool,
    pub(crate) cell_idx: usize,
    pub(crate) indices: USizeVec3,
}

#[derive(Clone)]
struct Cell {
    dyn_handles: Vec<usize>,
    static_handles: Vec<usize>,
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

    fn remove_static(&mut self, proxy_idx: usize) {
        if let Some(pos) = self
            .static_handles
            .iter()
            .copied()
            .position(|x| x == proxy_idx)
        {
            self.static_handles.remove(pos);
        }
    }

    fn remove_dyn(&mut self, proxy_idx: usize) {
        if let Some(pos) = self
            .dyn_handles
            .iter()
            .copied()
            .position(|x| x == proxy_idx)
        {
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
        _triangle: &TriangleShape,
        _tri_aabb: &Aabb,
        _triangle_index: usize,
    ) -> bool {
        self.hit = true;

        false
    }
}

struct CellGrid {
    max_pos: Vec3A,
    min_pos: Vec3A,
    cell_size: f32,
    cell_size_sq: f32,
    num_cells: USizeVec3,
    num_dyn_proxies: u32,
    cells: Vec<Cell>,
}

impl CellGrid {
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

    fn update_cells_static<const ADD: bool>(
        &mut self,
        proxy: &RsBroadphaseProxy,
        col_obj: &CollisionObject,
        proxy_idx: usize,
    ) {
        // TODO: "Fix dumb massive value aabb bug"
        let aabb_max = proxy.broadphase_proxy.aabb.max.min(self.max_pos);

        let min = self.get_cell_indices(proxy.broadphase_proxy.aabb.min);
        let max = self.get_cell_indices(aabb_max);

        let tri_mesh_shape = match col_obj.get_collision_shape() {
            Some(CollisionShapes::TriangleMesh(mesh)) => Some(mesh),
            _ => None,
        };

        let mut callback_inst = BoolHitTriangleCallback::default();
        let mut cells = Vec::with_capacity(27);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    debug_assert!(cells.is_empty());
                    if ADD && let Some(mesh_interface) = tri_mesh_shape {
                        let cell_min = self.get_cell_min_pos(USizeVec3::new(i, j, k));
                        let cell_aabb =
                            Aabb::new(cell_min, cell_min + Vec3A::splat(self.cell_size));

                        callback_inst.hit = false;
                        mesh_interface.process_all_triangles(&mut callback_inst, &cell_aabb);

                        if !callback_inst.hit {
                            continue;
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
                            for &static_handle in &self.cells[i].static_handles {
                                // check if static_handle and proxy are the same
                                if static_handle == proxy_idx {
                                    already_exists = true;
                                    break;
                                }
                            }

                            if !already_exists {
                                self.cells[i].static_handles.push(proxy_idx);
                            }
                        } else {
                            self.cells[i].remove_static(proxy_idx);
                        }
                    }

                    cells.clear();
                }
            }
        }
    }

    fn update_cells_dynamic<const ADD: bool>(&mut self, proxy_idx: usize, indices: USizeVec3) {
        let min = USizeVec3::ONE.max(indices) - USizeVec3::ONE;
        let max = (self.num_cells - USizeVec3::ONE).min(indices + USizeVec3::ONE);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    let cell = self.get_cell(USizeVec3::new(i, j, k));

                    if ADD {
                        cell.dyn_handles.push(proxy_idx);
                    } else {
                        cell.remove_dyn(proxy_idx);
                    }
                }
            }
        }
    }
}

pub struct RsBroadphase {
    cell_grid: CellGrid,
    min_dyn_handle_index: usize,
    pub handles: Vec<RsBroadphaseProxy>,
    pair_cache: HashedOverlappingPairCache,
}

impl RsBroadphase {
    #[must_use]
    pub fn new(
        min_pos: Vec3A,
        max_pos: Vec3A,
        cell_size: f32,
        pair_cache: HashedOverlappingPairCache,
    ) -> Self {
        debug_assert!(min_pos.cmple(max_pos).all(), "Invalid min/max pos");

        let range = max_pos - min_pos;
        let num_cells = (range / cell_size)
            .ceil()
            .as_usizevec3()
            .max(USizeVec3::ONE);
        let total_cells = num_cells.element_product();
        let cells = vec![Cell::default(); total_cells];

        Self {
            min_dyn_handle_index: 0,
            cell_grid: CellGrid {
                max_pos,
                min_pos,
                cell_size,
                cell_size_sq: cell_size * cell_size,
                num_cells,
                num_dyn_proxies: 0,
                cells,
            },
            handles: Vec::with_capacity(32),
            pair_cache,
        }
    }

    pub fn set_aabb(&mut self, col_obj: &CollisionObject, proxy_idx: usize, aabb: Aabb) {
        let sbp = &mut self.handles[proxy_idx];

        if sbp.broadphase_proxy.aabb.min != aabb.min || sbp.broadphase_proxy.aabb.max != aabb.max {
            if sbp.is_static {
                self.cell_grid
                    .update_cells_static::<false>(sbp, col_obj, proxy_idx);

                sbp.broadphase_proxy.aabb = aabb;

                self.cell_grid
                    .update_cells_static::<true>(sbp, col_obj, proxy_idx);
            } else {
                let old_index = sbp.cell_idx;
                sbp.broadphase_proxy.aabb = aabb;

                let new_indices = self.cell_grid.get_cell_indices(aabb.min);
                let new_index = self.cell_grid.cell_indices_to_index(new_indices);
                self.handles[proxy_idx].cell_idx = new_index;

                if new_index != old_index && self.cell_grid.num_dyn_proxies > 1 {
                    self.cell_grid
                        .update_cells_dynamic::<false>(proxy_idx, new_indices);
                    self.handles[proxy_idx].indices = new_indices;
                    self.cell_grid
                        .update_cells_dynamic::<true>(proxy_idx, new_indices);
                }
            }
        }
    }

    pub fn create_proxy(
        &mut self,
        aabb: Aabb,
        co: &CollisionObject,
        collision_filter_group: i32,
        collision_filter_mask: i32,
    ) -> usize {
        debug_assert!(aabb.min.cmple(aabb.max).all());

        let is_static = co.is_static_object();
        let world_index = co.get_world_array_index();

        let new_handle_idx = self.handles.len();
        let indices = self.cell_grid.get_cell_indices(aabb.min);
        let cell_idx = self.cell_grid.cell_indices_to_index(indices);

        let new_handle = RsBroadphaseProxy {
            broadphase_proxy: BroadphaseProxy {
                aabb,
                client_object_idx: Some(world_index),
                collision_filter_group,
                collision_filter_mask,
                unique_id: new_handle_idx + 2,
            },
            is_static,
            cell_idx,
            indices,
        };

        if is_static {
            if self.cell_grid.num_dyn_proxies == 0 {
                self.min_dyn_handle_index = new_handle_idx + 1;
            }

            self.cell_grid
                .update_cells_static::<true>(&new_handle, co, new_handle_idx);
        } else {
            debug_assert!(aabb.min.distance_squared(aabb.max) <= self.cell_grid.cell_size_sq);

            self.cell_grid
                .update_cells_dynamic::<true>(new_handle_idx, indices);
            self.cell_grid.num_dyn_proxies += 1;
        }

        self.handles.push(new_handle);
        new_handle_idx
    }

    pub fn calculate_overlapping_pairs(&mut self) {
        debug_assert!(self.pair_cache.is_empty());
        if self.cell_grid.num_dyn_proxies == 0 {
            return;
        }

        for (i, proxy) in self
            .handles
            .iter()
            .enumerate()
            .skip(self.min_dyn_handle_index)
            .filter(|(_, proxy)| !proxy.is_static)
        {
            let cell = &self.cell_grid.cells[proxy.cell_idx];
            for &other_proxy_idx in &cell.static_handles {
                let other_proxy = &self.handles[other_proxy_idx];

                if test_aabb_against_aabb(
                    &proxy.broadphase_proxy.aabb,
                    &other_proxy.broadphase_proxy.aabb,
                ) && !self.pair_cache.contains_pair(proxy, other_proxy)
                {
                    self.pair_cache
                        .add_overlapping_pair(proxy, i, other_proxy, other_proxy_idx);
                }
            }

            if self.cell_grid.num_dyn_proxies > 1 && !cell.dyn_handles.is_empty() {
                for &other_proxy_idx in &cell.dyn_handles {
                    if i == other_proxy_idx {
                        continue;
                    }

                    let other_proxy = &self.handles[other_proxy_idx];
                    if test_aabb_against_aabb(
                        &proxy.broadphase_proxy.aabb,
                        &other_proxy.broadphase_proxy.aabb,
                    ) && !self.pair_cache.contains_pair(proxy, other_proxy)
                    {
                        self.pair_cache.add_overlapping_pair(
                            proxy,
                            i,
                            other_proxy,
                            other_proxy_idx,
                        );
                    }
                }
            }
        }
    }

    pub fn process_all_overlapping_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objects: &[Rc<RefCell<RigidBody>>],
        dispatcher: &mut CollisionDispatcher,
        contact_added_callback: &mut T,
    ) {
        self.pair_cache.process_all_overlapping_pairs(
            collision_objects,
            dispatcher,
            &self.handles,
            contact_added_callback,
        );
    }

    pub fn ray_test<T: BroadphaseAabbCallback>(
        &self,
        ray_from: Vec3A,
        ray_to: Vec3A,
        ray_callback: &mut T,
    ) {
        debug_assert!(ray_from.distance_squared(ray_to) < self.cell_grid.cell_size_sq);
        let cell = &self.cell_grid.cells[self.cell_grid.get_cell_index(ray_from)];

        for &other_proxy_idx in cell.static_handles.iter().chain(&cell.dyn_handles) {
            let other_proxy = &self.handles[other_proxy_idx].broadphase_proxy;
            if other_proxy.client_object_idx.is_some() {
                ray_callback.process(other_proxy);
            }
        }
    }
}
