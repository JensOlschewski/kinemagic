/// Time-grid parameters for solver evaluation.
///
/// [`SolverConfig::times`] begins at `start_time` and progresses by `step_size`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolverConfig {
    start_time: f64,
    end_time: f64,
    step_size: f64,
}

impl SolverConfig {
    /// Creates a time grid from `start_time` to `end_time` in `step_size` steps.
    pub fn new(start_time: f64, end_time: f64, step_size: f64) -> Self {
        Self {
            start_time,
            end_time,
            step_size,
        }
    }

    /// Returns default one-step time grid from `0.0` to `1.0`.
    pub const fn defaults() -> Self {
        Self {
            start_time: 0.0,
            end_time: 1.0,
            step_size: 1.0,
        }
    }

    /// Iterates time-grid points, including `start_time`.
    ///
    /// Point count rounds `(end_time - start_time) / step_size` to nearest integer.
    /// Final point can differ from `end_time`.
    pub fn times(&self) -> impl Iterator<Item = f64> {
        let count = ((self.end_time - self.start_time) / self.step_size).round() as usize;
        (0..=count).map(move |index| self.start_time + index as f64 * self.step_size)
    }

    pub fn start_time(&self) -> f64 {
        self.start_time
    }
    pub fn end_time(&self) -> f64 {
        self.end_time
    }
    pub fn step_size(&self) -> f64 {
        self.step_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_has_one_step_from_zero_to_one() {
        let config = SolverConfig::defaults();

        assert_eq!(config.start_time(), 0.0);
        assert_eq!(config.end_time(), 1.0);
        assert_eq!(config.step_size(), 1.0);
        assert_eq!(config.times().collect::<Vec<_>>(), vec![0.0, 1.0]);
    }

    #[test]
    fn times_generates_expected_grid_for_nonzero_start_and_fractional_step() {
        let config = SolverConfig::new(1.0, 2.0, 0.25);

        assert_eq!(
            config.times().collect::<Vec<_>>(),
            vec![1.0, 1.25, 1.5, 1.75, 2.0],
        );
    }
}
