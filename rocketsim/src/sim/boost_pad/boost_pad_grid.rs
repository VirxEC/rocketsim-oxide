use glam::{USizeVec2, Vec2, Vec3A};

use crate::shared::bvh::SimpleNodeProcessor;
use crate::shared::{Aabb, bvh};
use crate::{BoostPad, BoostPadConfig, CarState, MutatorConfig, consts::boost_pads};

#[derive(Debug, Clone)]
pub(crate) struct BoostPadGrid {
    bvh_tree: bvh::Tree,
    all_pads: Vec<BoostPad>,
    max_pad_z: f32,
}

impl BoostPadGrid {
    #[must_use]
    pub fn new(pad_configs: &Vec<BoostPadConfig>) -> Self {
        assert!(!pad_configs.is_empty());

        let all_pads: Vec<BoostPad> = pad_configs.clone().into_iter().map(BoostPad::new).collect();

        let all_aabb = {
            let mut all_aabb_accum: Option<Aabb> = None;

            for pad in &all_pads {
                let pad_aabb = pad.aabb();
                if let Some(all_aabb) = all_aabb_accum {
                    all_aabb_accum = Some(all_aabb.combine(&pad_aabb));
                } else {
                    all_aabb_accum = Some(pad_aabb);
                }
            }
            all_aabb_accum.unwrap()
        };

        let mut bvh_tree = bvh::Tree::new(all_aabb, pad_configs.len());

        let mut aabb_nodes = Vec::new();
        for (i, pad) in all_pads.iter().enumerate() {
            let node = bvh::Node {
                aabb: pad.aabb(),
                node_type: bvh::BvhNodeType::Leaf { leaf_index: i },
            };
            aabb_nodes.push(node);
        }

        let num_nodes = aabb_nodes.len();
        bvh_tree.build_tree(&mut aabb_nodes, 0, num_nodes);

        Self {
            bvh_tree,
            all_pads,
            max_pad_z: all_aabb.max.z,
        }
    }

    #[must_use]
    pub fn pads(&self) -> &[BoostPad] {
        &self.all_pads
    }

    #[must_use]
    pub fn pads_mut(&mut self) -> &mut [BoostPad] {
        &mut self.all_pads
    }

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

        let car_center_aabb = Aabb::new(car_state.pos, car_state.pos);
        let mut node_processor = SimpleNodeProcessor::new();
        self.bvh_tree
            .report_aabb_overlapping_node(&mut node_processor, &car_center_aabb);

        for pad_idx in node_processor.leaf_indices {
            let pad = &mut self.all_pads[pad_idx];

            if let Some(last_give_tick_count) = pad.gave_boost_tick_count
                && (tick_count - last_give_tick_count) as f32 * tick_time
                    < pad.config.get_max_cooldown(mutator_config)
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
                car_state.boost = (car_state.boost + pad.config.get_boost_amount(mutator_config))
                    .min(mutator_config.car_max_boost_amount);
                pad.gave_boost_tick_count = Some(tick_count);
                return;
            }
        }
    }
}
