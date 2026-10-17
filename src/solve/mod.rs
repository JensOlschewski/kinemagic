//! Kinematic solving of a prepared [`Model`].
//!
//! - [`tree`]: forward kinematics over the spanning tree (poses, body Jacobians).
//! - [`closure`]: loop-closure residuals, Jacobians, and the Newton solve.
//! - [`SequenceSolver`]: stateful entry point that dispatches between the two.

pub mod closure;
pub mod state;
pub mod tree;

use thiserror::Error;

use crate::data::coordinates::GeneralizedCoordinates;
use crate::model::Model;
use closure::newton::solve_closed_loop;

pub use crate::model::mechanism::joint::geometry::child_pose;
pub use crate::solve::closure::analysis::{
    ClosureJacobian, JacobianError, Mobility, MobilityError, analyze_closure_jacobian_at,
    mobility_at,
};
pub use crate::solve::closure::{
    ResidualError, closure_jacobian_for_configuration_at, closure_orientation_residual_rates,
    closure_orientation_residuals_at, closure_position_residual_rates, closure_position_residuals,
    closure_residual_rates, closure_residuals_at,
};
pub use crate::solve::state::{
    BodyJacobian, BodyJacobians, BodyPose, BodyPoses, BodyTwist, TreeBodyJacobians,
    TreeTwistColumns,
};
pub use crate::solve::tree::{
    evaluate_tree_poses_at, tree_body_jacobians_for_configuration_at,
    tree_poses_for_configuration_at, tree_twist_columns_for_configuration_at,
};

/// Solves mechanism configurations at successive evaluation times.
///
/// Reuses the last successful solution as the initial guess for free
/// joint coordinates. A failed solve leaves the stored solution unchanged.
pub struct SequenceSolver<'a> {
    problem: &'a Model,
    candidates: GeneralizedCoordinates,
    has_previous_solution: bool,
}

impl<'a> SequenceSolver<'a> {
    /// Creates a solver for the prepared problem.
    ///
    /// The first solve starts with zero values for free joint coordinates.
    pub fn new(problem: &'a Model) -> Self {
        Self {
            problem,

            // Joint-coordinate values retained from the last successful solve.
            candidates: GeneralizedCoordinates::new(problem.coordinate_layout()),

            // Distinguishes initial solving from continuation state, even when
            // the configuration is zero.
            has_previous_solution: false,
        }
    }

    /// Solves the mechanism configuration at `time`.
    ///
    /// Reuses the last successful joint-coordinate solution as the initial
    /// guess. Evaluation times do not need to be increasing.
    ///
    /// # Errors
    ///
    /// Returns an error if `time` is not finite or the closed-loop solve fails.
    /// A failed solve leaves the stored solution unchanged.
    pub fn solve_at(&mut self, time: f64) -> Result<BodyPoses, SolverError> {
        let mut candidates = self.candidates.clone();
        let poses = solve_candidates(
            self.problem,
            time,
            &mut candidates,
            self.has_previous_solution,
        )?;
        self.candidates = candidates;
        self.has_previous_solution = true;
        Ok(poses)
    }
}

pub fn solve(problem: &Model) -> Result<BodyPoses, SolverError> {
    solve_at(problem, 0.0)
}

pub fn solve_at(problem: &Model, time: f64) -> Result<BodyPoses, SolverError> {
    SequenceSolver::new(problem).solve_at(time)
}

fn solve_candidates(
    problem: &Model,
    time: f64,
    candidates: &mut GeneralizedCoordinates,
    continuation: bool,
) -> Result<BodyPoses, SolverError> {
    validate_time(time)?;
    if problem.closure_joint_ids().is_empty() {
        Ok(tree_poses_for_configuration_at(problem, candidates, time)
            .expect("solver configuration matches problem layout"))
    } else {
        solve_closed_loop(problem, time, candidates, continuation)
    }
}

pub fn validate_time(time: f64) -> Result<(), SolverError> {
    if time.is_finite() {
        Ok(())
    } else {
        Err(SolverError::NonFiniteTime { time })
    }
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum SolverError {
    #[error("evaluation time is non-finite: {time}")]
    NonFiniteTime { time: f64 },
    #[error(transparent)]
    Jacobian(#[from] JacobianError),
    #[error("closed-loop residual is non-finite")]
    NonFiniteResidual,
    #[error("closed-loop Newton step is non-finite")]
    NonFiniteStep,
    #[error("closed-loop residual has no independent coordinates")]
    NoIndependentCoordinates,
    #[error(
        "closed-loop mechanism is under-determined: {free_coordinates} free coordinates, {dependent_coordinates} dependent coordinates, {remaining_dofs} remaining DOFs"
    )]
    UnderDetermined {
        free_coordinates: usize,
        dependent_coordinates: usize,
        remaining_dofs: usize,
    },
    #[error("closed-loop linear solve failed: {0}")]
    LinearSolveFailed(&'static str),
    #[error(
        "closed-loop solve did not converge after {iterations} iterations (residual norm {residual_norm:e})"
    )]
    NonConvergent {
        iterations: usize,
        residual_norm: f64,
    },
}

#[cfg(test)]
mod tests {
    use crate::io::yaml::parse_yaml_str;

    use super::*;
    #[test]
    fn sequence_solver_commits_candidates_only_after_success() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let mut solver = SequenceSolver::new(&problem);
        solver.solve_at(0.0).unwrap();
        let candidates = solver.candidates.clone();

        assert!(solver.solve_at(f64::NAN).is_err());
        assert_eq!(solver.candidates, candidates);
    }
}
