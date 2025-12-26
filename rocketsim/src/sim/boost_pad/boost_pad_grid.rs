use crate::consts::boost_pads;
use crate::{BoostPad, BoostPadConfig, CarState, MutatorConfig};
use glam::{USizeVec2, Vec2, Vec3A};

#[derive(Debug, Clone)]
struct GridCell {
    pub pad_indices: Vec<usize>,
}

impl Default for GridCell {
    fn default() -> Self {
        Self {
            pad_indices: Vec::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct BoostPadGrid {
    cells: [GridCell; BoostPadGrid::CELL_AMOUNT],
    all_pads: Vec<BoostPad>,
    max_pad_z: f32,
}

impl BoostPadGrid {
    const GRID_EXTENT: Vec2 = Vec2::new(4096.0, 5120.0);
    const GRID_SIZE: Vec2 = Vec2::new(Self::GRID_EXTENT.x * 2.0, Self::GRID_EXTENT.y * 2.0);
    const CELL_COUNTS: USizeVec2 = USizeVec2::new(8, 10);
    const CELL_AMOUNT: usize = Self::CELL_COUNTS.x * Self::CELL_COUNTS.y;
    const CELL_SIZE: Vec2 = Vec2::new(
        Self::GRID_SIZE.x / Self::CELL_COUNTS.x as f32,
        Self::GRID_SIZE.y / Self::CELL_COUNTS.y as f32,
    );

    pub fn new(pad_configs: &Vec<BoostPadConfig>) -> Self {
        let mut cells: [GridCell; Self::CELL_AMOUNT] = std::array::from_fn(|_| GridCell::default());
        let mut all_pads: Vec<BoostPad> = Vec::new();
        for pad_config in pad_configs {
            let pad_pos = pad_config.pos;
            let boost_pad = BoostPad::new(*pad_config);
            let pad_idx = all_pads.len();
            all_pads.push(boost_pad);

            // TODO: Inefficient, scaling is O(num_cell*num_pads) :(
            const BOOST_PAD_MAX_RAD: f32 =
                f32::max(boost_pads::BOX_RAD_BIG, boost_pads::BOX_RAD_SMALL);
            let (pad_aabb_min, pad_aabb_max) = (
                pad_pos.truncate() - BOOST_PAD_MAX_RAD,
                pad_pos.truncate() + BOOST_PAD_MAX_RAD,
            );
            let mut overlapped_any = false;
            for (cell_idx, cell) in cells.iter_mut().enumerate() {
                let (cell_aabb_min, cell_aabb_max) = Self::calc_cell_aabb_2d(cell_idx);
                if pad_aabb_min.cmple(cell_aabb_max).all()
                    && pad_aabb_max.cmpge(cell_aabb_min).all()
                {
                    overlapped_any = true;
                    cell.pad_indices.push(pad_idx);
                }
            }

            if !overlapped_any {
                panic!("Boost pad at location {pad_pos} is completely outside of the grid space");
            }
        }

        let mut max_pad_z = -f32::MAX;
        for config in pad_configs {
            max_pad_z = max_pad_z.max(config.pos.z);
        }

        Self {
            cells,
            all_pads,
            max_pad_z,
        }
    }

    fn calc_cell_idx(pos: Vec3A) -> Option<usize> {
        let pos_2d = pos.truncate();
        let idx_2d = (pos_2d / Self::CELL_SIZE).as_ivec2() + (Self::CELL_COUNTS / 2).as_ivec2();
        if idx_2d.x < 0 || idx_2d.y < 0 {
            return None;
        }
        let idx_2d_u = idx_2d.as_usizevec2();
        if idx_2d_u.x >= Self::CELL_COUNTS.x || idx_2d_u.y >= Self::CELL_COUNTS.y {
            return None;
        }

        Some(idx_2d_u.x + (idx_2d_u.y * Self::CELL_COUNTS.x))
    }

    fn calc_cell_aabb_2d(cell_idx: usize) -> (Vec2, Vec2) {
        let idx_2d = USizeVec2::new(
            cell_idx % Self::CELL_COUNTS.x,
            cell_idx / Self::CELL_COUNTS.x,
        );
        let offset = idx_2d.as_vec2() * Self::CELL_SIZE;
        let start_pos_2d = offset - (Self::GRID_SIZE / 2.0);
        (start_pos_2d, start_pos_2d + Self::CELL_SIZE)
    }

    pub fn pads(&self) -> &[BoostPad] {
        &self.all_pads
    }

    fn get_cell_at_pos(&self, pos: Vec3A) -> Option<&GridCell> {
        if let Some(cell_idx) = Self::calc_cell_idx(pos) {
            Some(&self.cells[cell_idx])
        } else {
            None
        }
    }

    /// Returns true if boost was given
    pub(crate) fn maybe_give_car_boost(
        &mut self,
        car_state: &mut CarState,
        mutator_config: &MutatorConfig,
    ) -> bool {
        if car_state.boost >= mutator_config.car_max_boost_amount {
            return false; // Already full on boost
        }

        if car_state.pos.z > self.max_pad_z {
            return false; // Can't possibly overlap with a boost pad
        }

        if let Some(cell) = self.get_cell_at_pos(car_state.pos) {
            for pad_idx_ref in &cell.pad_indices {
                let mut pad = self.all_pads[*pad_idx_ref];
                let mut pad_state = *pad.get_state();
                if !pad_state.gave_car_boost && pad_state.cooldown == 0.0 {

                    // Check if car origin is inside the cylinder hitbox
                    let pad_pos = pad.get_config().pos;
                    let cyl_radius = pad.get_radius();
                    let dist_sq_2d =
                        Vec2::distance_squared(pad_pos.truncate(), car_state.pos.truncate());
                    let overlapping = dist_sq_2d < (cyl_radius * cyl_radius)
                        && (car_state.pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT;
                    if overlapping { // Give boost

                        let max_cooldown = if pad.get_config().is_big {
                            mutator_config.boost_pad_cooldown_big
                        } else {
                            mutator_config.boost_pad_cooldown_small
                        };

                        let boost_give_amount = if pad.get_config().is_big {
                            mutator_config.boost_pad_amount_big
                        } else {
                            mutator_config.boost_pad_amount_small
                        };

                        car_state.boost = (car_state.boost + boost_give_amount)
                            .min(mutator_config.car_max_boost_amount);
                        pad_state.cooldown = max_cooldown;
                        pad.set_state(pad_state);
                        return true;
                    }
                }
            }
        }

        false
    }

    pub(crate) fn post_tick_update(&mut self, tick_time: f32) {
        for pad in self.all_pads.iter_mut() {
            let mut pad_state = *pad.get_state();
            pad_state.cooldown = (pad_state.cooldown - tick_time).max(0.0);
            pad_state.gave_car_boost = false;
            pad.set_state(pad_state);
        }
    }
}
