use super::{
    broadphase_interface::BroadphaseInterface,
    broadphase_proxy::{BroadphaseNativeTypes, BroadphaseProxy},
    overlapping_pair_cache::OverlappingPairCache,
};
use crate::collision::{
    dispatch::collision_object::CollisionObject,
    shapes::{
        collision_shape::CollisionShapes, triangle_callback::TriangleCallback,
        triangle_mesh_shape::TriangleMeshShape,
    },
};
use glam::{USizeVec3, Vec3A};
use std::{cell::RefCell, rc::Rc};

#[derive(Clone, Default)]
pub struct RsBroadphaseProxy {
    broadphase_proxy: Rc<RefCell<BroadphaseProxy>>,
    is_static: bool,
    cell_idx: usize,
    indices: USizeVec3,
    shape_type: BroadphaseNativeTypes,
    next_free: usize,
}

impl RsBroadphaseProxy {
    pub fn set_next_free(&mut self, next: i32) {
        self.broadphase_proxy.borrow_mut().unique_id = next;
    }

    pub fn get_next_free(&self) -> i32 {
        self.broadphase_proxy.borrow().unique_id
    }
}

#[derive(Clone)]
struct Cell {
    dyn_handles: Vec<Rc<RefCell<RsBroadphaseProxy>>>,
    static_handles: Vec<Rc<RefCell<RsBroadphaseProxy>>>,
}

impl Default for Cell {
    fn default() -> Self {
        Cell {
            dyn_handles: Vec::with_capacity(Cell::RESERVED_SIZE),
            static_handles: Vec::with_capacity(Cell::RESERVED_SIZE),
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
    num_dyn_proxies: i32,
    total_static_pairs: i32,
    total_dyn_pairs: i32,
    total_iters: i32,
    // std::vector<std::pair<btRSBroadphaseProxy*, btRSBroadphaseProxy*>> activePairs;
    cells: Vec<Cell>,
    handles: Vec<Rc<RefCell<RsBroadphaseProxy>>>,
    first_free_handle: usize,
    pair_cache: Box<dyn OverlappingPairCache>,
    // bool m_ownsPairCache;
    // int m_invalidPair;
}

impl RsBroadphase {
    pub fn new(
        min_pos: Vec3A,
        max_pos: Vec3A,
        cell_size: f32,
        pair_cache: Box<dyn OverlappingPairCache>,
        max_handles: usize,
    ) -> Self {
        debug_assert!(min_pos.cmple(max_pos).all(), "Invalid min/max pos");

        let handles: Vec<Rc<RefCell<RsBroadphaseProxy>>> = vec![Rc::default(); max_handles];
        for (i, handle) in handles.iter().enumerate() {
            handle.borrow_mut().set_next_free(i as i32 + 1);
            handle.borrow_mut().broadphase_proxy.borrow_mut().unique_id = i as i32 + 2;
        }
        handles.last().unwrap().borrow_mut().set_next_free(0);

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
        self.first_free_handle = self.handles[free_handle].borrow().get_next_free() as usize;
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

    fn cell_indices_to_index(&self, indices: USizeVec3) -> usize {
        indices.x * self.num_cells.y * self.num_cells.z + indices.y * self.num_cells.z + indices.z
    }

    fn get_cell_min_pos(&self, indices: USizeVec3) -> Vec3A {
        self.min_pos + indices.as_vec3a() * self.cell_size
    }

    fn get_cell(&mut self, indices: USizeVec3) -> &mut Cell {
        let idx = self.cell_indices_to_index(indices);
        &mut self.cells[idx]
    }

    fn update_cells_static<const ADD: bool>(&mut self, proxy: Rc<RefCell<RsBroadphaseProxy>>) {
        let proxy_ref = proxy.borrow();
        let inner_proxy = &*proxy_ref.broadphase_proxy.borrow();

        // TODO: "Fix dumb massive value aabb bug"
        let aabb_max = inner_proxy.aabb_max.min(self.max_pos);

        let min = self.get_cell_indices(inner_proxy.aabb_min);
        let max = self.get_cell_indices(aabb_max);

        let col_obj = &*inner_proxy.client_object.borrow();
        let obj = col_obj.get_collision_shape().as_ref().unwrap().borrow();
        let tri_mesh_shape = match &*obj {
            CollisionShapes::TriangleMesh(mesh) => Some(&*mesh.mesh_interface),
            _ => None,
        };

        let mut callback_inst = BoolHitTriangleCallback::default();
        let mut cells = Vec::with_capacity(27);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    debug_assert!(cells.is_empty());

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

                    if ADD {
                        if let Some(mesh_interface) = tri_mesh_shape {
                            let cell_min = self.get_cell_min_pos(USizeVec3::new(i, j, k));
                            let cell_max = cell_min + Vec3A::splat(self.cell_size);

                            callback_inst.hit = false;
                            TriangleMeshShape::process_all_triangles(
                                mesh_interface,
                                &mut callback_inst,
                                cell_min,
                                cell_max,
                            );

                            if !callback_inst.hit {
                                cells.clear();
                                continue;
                            }
                        }
                    }

                    for i in cells.drain(..) {
                        if ADD {
                            let mut already_exists = false;
                            for static_handle in &self.cells[i].static_handles {
                                // check if static_handle and proxy are the same
                                if Rc::ptr_eq(static_handle, &proxy) {
                                    already_exists = true;
                                    break;
                                }
                            }

                            if !already_exists {
                                self.cells[i].static_handles.push(proxy.clone());
                            }
                        } else {
                            self.cells[i].remove_static(&proxy);
                        }
                    }
                }
            }
        }
    }

    fn update_cells_dynamic<const ADD: bool>(
        &mut self,
        proxy: Rc<RefCell<RsBroadphaseProxy>>,
        indices: USizeVec3,
    ) {
        let min = USizeVec3::ONE.max(indices) - USizeVec3::ONE;
        let max = (self.num_cells - USizeVec3::ONE).min(indices + USizeVec3::ONE);

        for i in min.x..=max.x {
            for j in min.y..=max.y {
                for k in min.z..=max.z {
                    let cell = self.get_cell(USizeVec3::new(i, j, k));

                    if ADD {
                        cell.dyn_handles.push(proxy.clone());
                    } else {
                        cell.remove_dyn(&proxy);
                    }
                }
            }
        }
    }
}

impl BroadphaseInterface for RsBroadphase {
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
    ) -> Rc<RefCell<BroadphaseProxy>> {
        debug_assert!(aabb_min.cmple(aabb_max).all());

        let is_static = collision_object.borrow().is_static_object();
        let new_handle_idx = self.alloc_handle();
        let cell_idx = self.get_cell_index(aabb_min);
        let indices = self.get_cell_indices(aabb_min);

        let new_handle = Rc::new(RefCell::new(RsBroadphaseProxy {
            broadphase_proxy: Rc::new(RefCell::new(BroadphaseProxy {
                aabb_min,
                aabb_max,
                client_object: collision_object,
                collision_filter_group,
                collision_filter_mask,
                unique_id: 0,
            })),
            is_static,
            cell_idx,
            indices,
            shape_type,
            next_free: 0,
        }));

        if is_static {
            self.update_cells_static::<true>(new_handle.clone());
        } else {
            debug_assert!(aabb_min.distance_squared(aabb_max) <= self.cell_size_sq);

            self.update_cells_dynamic::<true>(new_handle.clone(), indices);
            self.num_dyn_proxies += 1;
        }

        let inner_proxy = new_handle.borrow().broadphase_proxy.clone();
        self.handles[new_handle_idx] = new_handle;
        inner_proxy
    }
}
