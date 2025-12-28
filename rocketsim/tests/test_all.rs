#![allow(unused)]

mod comparison_test;

use crate::comparison_test::*;

fn init_for_test() {
    rocketsim::init("../collision_meshes", false).unwrap();
    rocketsim_rs::init(Some("../collision_meshes"), false);
}

#[test]
fn test_comparisons() {
    const FAIL_ERROR_THRESH: f32 = 1.0; // Adjust as needed

    init_for_test();

    let test_cases = all_test_cases::make_all_cases();
    for test_case in test_cases {
        let test_result = test_case.run();

        for (tick_idx, state) in test_result.ticks.iter().enumerate() {
            let comparison = &state.comparison;
            for (value_name, value_err) in &comparison.combine_all_err_sets() {
                if *value_err >= FAIL_ERROR_THRESH {
                    let value_err_stat = test_result.val_err_stats.get(value_name).unwrap();
                    let case_name = test_case.name;
                    let tick_number = tick_idx + 1;
                    let mean_err = value_err_stat.mean();

                    let prev_state = &test_result.ticks[tick_idx.max(1) - 1];
                    let cur_state = &test_result.ticks[tick_idx];

                    panic!(
                        "Test comparison case \"{case_name}\" failed at tick #{tick_number} with value \"{value_name}\" (scaled error = {value_err})\
                        \n\tError mean for \"{value_name}\" during test case \"{case_name}\" = {mean_err}\
                        \nPrev state: {prev_state}\
                        \nResulting state: {cur_state}"
                    );
                }
            }
        }
    }
}
