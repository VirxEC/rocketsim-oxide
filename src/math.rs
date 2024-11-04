pub struct LinearPieceCurve<const N: usize> {
    pub value_mappings: [(f32, f32); N],
}

impl<const N: usize> LinearPieceCurve<N> {
    /// Returns the output of the curve
    ///
    /// # Arguments
    ///
    /// * `input` - The input to the curve
    /// * `default_output` - The default output if N is 0
    #[must_use]
    pub fn get_output(&self, input: f32, default_output: Option<f32>) -> f32 {
        if N == 0 {
            return default_output.unwrap_or(1.);
        }

        let first_val_pair = self.value_mappings[0];

        if input <= first_val_pair.0 {
            return first_val_pair.1;
        }

        for i in 1..N {
            let after_pair = self.value_mappings[i];
            let before_pair = self.value_mappings[i - 1];

            if after_pair.0 > input {
                let range_between = after_pair.0 - before_pair.0;
                let val_diff_between = after_pair.1 - before_pair.1;
                let linear_interp_factor = (input - before_pair.0) / range_between;
                return before_pair.1 + val_diff_between * linear_interp_factor;
            }
        }

        self.value_mappings[N - 1].1
    }
}
