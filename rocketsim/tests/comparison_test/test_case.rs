use crate::comparison_test::setup::{BallSetup, CarSetup};
use crate::comparison_test::state_compare::StateComparison;
use crate::comparison_test::state_convert::{OldArena, OldArenaConfig, OldArenaPtr, OldCarConfig};
use crate::comparison_test::{TestResult, TestResultState, state_compare, state_convert};
use glam::Vec3A;
use log::info;
use rocketsim::*;

#[derive(Debug, Clone)]
pub struct TestCase {
    pub name: &'static str,
    pub game_mode: GameMode,
    pub car_setups: Vec<CarSetup>,
    pub ball_setup: Option<BallSetup>,
    pub duration_ticks: usize,
}

impl TestCase {
    pub fn is_valid(&self) -> bool {
        !self.car_setups.is_empty() || !self.ball_setup.is_none()
    }

    fn make_new_old_arenas(&self) -> (Arena, OldArenaPtr) {
        assert!(self.is_valid());

        let mut new_arena = Arena::new(self.game_mode);
        let mut old_arena_ptr = OldArena::new(
            state_convert::conv_to_old_game_mode(self.game_mode),
            OldArenaConfig::default(),
            120,
        );

        {
            // Set cars
            for car_setup in &self.car_setups {
                {
                    let new_car_id = new_arena.add_car(car_setup.team, CarConfig::OCTANE);
                    new_arena.get_car_mut(new_car_id).unwrap().controls = car_setup.controls;
                }
                {
                    let old_car_id = old_arena_ptr.pin_mut().add_car(
                        state_convert::conv_to_old_team(car_setup.team),
                        OldCarConfig::octane(),
                    );
                    old_arena_ptr
                        .pin_mut()
                        .set_car_controls(
                            old_car_id,
                            state_convert::conv_to_old_car_controls(car_setup.controls),
                        )
                        .unwrap();
                }
            }

            for (i, car_setup) in self.car_setups.iter().enumerate() {
                let new_car_id = (i + 1) as u64;
                let old_car_id = (i + 1) as u32;

                let new_car_state = car_setup.make_car_state();
                let old_car_state = state_convert::conv_to_old_car_state(&new_car_state);

                new_arena.set_car_state(new_car_id, car_setup.make_car_state());
                old_arena_ptr
                    .pin_mut()
                    .set_car(old_car_id, old_car_state)
                    .unwrap();
            }
        }

        if let Some(ball_setup) = self.ball_setup {
            // Set ball
            let new_ball_state = ball_setup.make_ball_state();
            new_arena.set_ball(new_ball_state);
            old_arena_ptr
                .pin_mut()
                .set_ball(state_convert::conv_to_old_ball_state(&new_ball_state));
        } else {
            // Send ball to the backrooms
            let mut new_ball_state = BallState::default();
            new_ball_state.phys.pos = Vec3A::new(0.0, 0.0, -9999.0);
            new_arena.set_ball(new_ball_state);
            old_arena_ptr
                .pin_mut()
                .set_ball(state_convert::conv_to_old_ball_state(&new_ball_state));
        }

        (new_arena, old_arena_ptr)
    }

    /// continuous_state_set: Will set the new arena's state to the old arena after every step.
    ///     This will better capture the moments of failure instead of the resulting divergence.
    pub fn run(&self, continuous_state_set: bool) -> TestResult {
        info!(
            "Running test case \"{}\" for \"{}\" ticks...",
            self.name, self.duration_ticks
        );

        let mut ticks = Vec::new();

        let (mut new_arena, mut old_arena_ptr) = self.make_new_old_arenas();
        for _ in 0..self.duration_ticks {
            new_arena.step(1);
            old_arena_ptr.pin_mut().step(1);

            let comparison = {
                let mut comparison = StateComparison::default();
                for i in 0..self.car_setups.len() {
                    let new_car_id = (i + 1) as u64;
                    let old_car_id = (i + 1) as u32;
                    let new_car_state = new_arena.get_car(new_car_id).unwrap().get_state();
                    let old_car_state = &old_arena_ptr.pin_mut().get_car(old_car_id);
                    let old_car_state_conv = &state_convert::conv_to_new_car_state(old_car_state);
                    let err_set = state_compare::map_car_err(new_car_state, old_car_state_conv);
                    comparison.car_errs.push(err_set);
                }

                if self.ball_setup.is_some() {
                    let new_ball_state = new_arena.get_ball();
                    let old_ball_state = &old_arena_ptr.pin_mut().get_ball();
                    let old_ball_state_conv =
                        &state_convert::conv_to_new_ball_state(old_ball_state);

                    let include_ball_rot = self.game_mode == GameMode::Snowday;
                    let err_set = state_compare::map_ball_err(
                        new_ball_state,
                        old_ball_state_conv,
                        include_ball_rot,
                    );

                    comparison.ball_err = Some(err_set);
                }
                comparison
            };

            let mut car_state_pairs = Vec::new();
            for i in 0..self.car_setups.len() {
                let car_id_new = (i + 1) as u64;
                let car_id_old = (i + 1) as u32;
                let new_car_state = *new_arena.get_car(car_id_new).unwrap().get_state();
                let old_car_state = old_arena_ptr.pin_mut().get_car(car_id_old);
                let old_car_state_conv = state_convert::conv_to_new_car_state(&old_car_state);
                if continuous_state_set {
                    new_arena.set_car_state(car_id_new, old_car_state_conv);
                }
                car_state_pairs.push((new_car_state, old_car_state_conv));
            }

            let ball_state_pair = if self.ball_setup.is_some() {
                let new_ball_state = *new_arena.get_ball();
                let old_ball_state = old_arena_ptr.pin_mut().get_ball();
                let old_ball_state_conv = state_convert::conv_to_new_ball_state(&old_ball_state);
                if continuous_state_set {
                    new_arena.set_ball(old_ball_state_conv);
                }
                Some((new_ball_state, old_ball_state_conv))
            } else {
                None
            };

            ticks.push(TestResultState {
                game_mode: self.game_mode,
                car_state_pairs,
                ball_state_pair,
                comparison,
            });
        }

        TestResult::new(ticks)
    }
}
