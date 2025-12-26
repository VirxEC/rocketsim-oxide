use glam::{USizeVec2, Vec2, Vec3A};

use crate::{BoostPad, BoostPadConfig, CarState, MutatorConfig, consts::boost_pads};

#[derive(Debug, Clone, Default)]
struct GridCell {
    pub pad_indices: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct BoostPadGrid {
    cells: [GridCell; BoostPadGrid::CELL_AMOUNT],
    pub(crate) all_pads: Vec<BoostPad>,
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

    #[must_use]
    pub fn new(pad_configs: &Vec<BoostPadConfig>) -> Self {
        let mut cells: [GridCell; Self::CELL_AMOUNT] = std::array::from_fn(|_| GridCell::default());
        let mut all_pads: Vec<BoostPad> = Vec::new();
        for pad_config in pad_configs {
            const BOOST_PAD_MAX_RAD: f32 = boost_pads::BOX_RAD_BIG.max(boost_pads::BOX_RAD_SMALL);

            let pad_pos = pad_config.pos;
            let boost_pad = BoostPad::new(*pad_config);
            let pad_idx = all_pads.len();
            all_pads.push(boost_pad);

            // TODO: Inefficient, scaling is O(num_cell*num_pads) :(
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

            assert!(
                overlapped_any,
                "Boost pad at location {pad_pos} is completely outside of the grid space"
            );
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

    #[must_use]
    pub fn pads(&self) -> &[BoostPad] { &self.all_pads }

    #[must_use]
    pub fn pads_mut(&mut self) -> &mut [BoostPad] { &mut self.all_pads }

    pub fn reset(&mut self) {
        for pad in &mut self.all_pads {
            pad.reset();
        }
    }

    /// Returns true if boost was given
    pub(crate) fn maybe_give_car_boost(
        &mut self,
        car_state: &mut CarState,
        mutator_config: &MutatorConfig,
        tick_count: u64,
        tick_time: f32,
    ) {
        if car_state.boost >= mutator_config.car_max_boost_amount {
            return; // Already full on boost
        }

        if car_state.pos.z > self.max_pad_z {
            return; // Can't possibly overlap with a boost pad
        }

        let Some(cell_idx) = Self::calc_cell_idx(car_state.pos)
        else { return; };

        let cell = &self.cells[cell_idx];

        for pad_idx_ref in &cell.pad_indices {
            let pad = &mut self.all_pads[*pad_idx_ref];

            let (cooldown, boost_give_amount) = if pad.config().is_big {
                (
                    mutator_config.boost_pad_cooldown_big,
                    mutator_config.boost_pad_amount_big,
                )
            } else {
                (
                    mutator_config.boost_pad_cooldown_small,
                    mutator_config.boost_pad_amount_small,
                )
            };

            if let Some(last_give_tick_count) = pad.internal_state.gave_boost_tick_count
                && (tick_count - last_give_tick_count) as f32 * tick_time < cooldown
            {
                continue;
            }

            // Check if car origin is inside the cylinder hitbox
            let pad_pos = pad.config().pos;
            let cyl_radius = pad.radius();
            let dist_sq_2d = pad_pos
                .truncate()
                .distance_squared(car_state.pos.truncate());
            let overlapping = dist_sq_2d < cyl_radius * cyl_radius
                && (car_state.pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT;
            if overlapping {
                // Give boost
                car_state.boost =
                    (car_state.boost + boost_give_amount).min(mutator_config.car_max_boost_amount);
                pad.internal_state.gave_boost_tick_count = Some(tick_count);

                return;
            }
        }
    }
}
