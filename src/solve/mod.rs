//! API Considerations
//!
//! BodyPose
//! - new
//! - position
//! - orientation
//! - marker_position
//! - marker_orientation
//!
//! BodyTwist
//! - new
//! - linear_velocity
//! - angular_velocity
//! - marker_linear_velocity
//!
//! KinematicSolver
//! - new
//! - solve(Option<time>)
//! - solve_with_progress(Option<time>)
//!
//! JointRelativeDisplacement (maybe define this in model or problem?)
//! -
//!
//!
//!
//! SolverFlow Considerations
//!
//! 1) tree_poses_for_candidates_at
//! - spherical_child_pose, dispatcher for this -> joint_child_pose (keep it simple)
//!
//! 2) closure_residuals (position level)
//!
//! Does it make sense to use one matrix here, which calculates all residuals at once?
//!
//! Matrix Construction vs. Single Solve
//!
//! 3) closure_residuals_rate (velocity level)
//!
//!

pub mod evaluation;
pub mod state;

use std::collections::BTreeMap;

use nalgebra::{DMatrix, DVector, Vector3};
use thiserror::Error;

use crate::data::coordinates::GeneralizedCoordinates;
use crate::model::Model;
use crate::model::mechanism::JointId;
use evaluation::{closure_jacobian_for_columns_at, closure_residuals_at};

pub use crate::model::mechanism::joint::spherical::spherical_child_pose;
pub use crate::solve::evaluation::{
    ResidualError, closure_jacobian, closure_jacobian_for_candidates,
    closure_jacobian_for_candidates_at, closure_jacobian_for_configuration,
    closure_jacobian_for_configuration_at, closure_orientation_residual_rates,
    closure_orientation_residuals, closure_position_residual_rates, closure_position_residuals,
    closure_residual_rates, closure_residuals, evaluate_tree_poses_at, tree_body_jacobians,
    tree_body_jacobians_for_candidates, tree_body_jacobians_for_candidates_at,
    tree_body_jacobians_for_configuration, tree_body_jacobians_for_configuration_at, tree_poses,
    tree_poses_at, tree_poses_for_candidates, tree_poses_for_candidates_at,
    tree_poses_for_configuration, tree_poses_for_configuration_at, tree_twist_columns,
    tree_twist_columns_for_candidates, tree_twist_columns_for_candidates_at,
    tree_twist_columns_for_configuration, tree_twist_columns_for_configuration_at,
};
pub use crate::solve::state::{
    BodyJacobian, BodyJacobians, BodyPose, BodyPoses, BodyTwist, TreeBodyJacobians,
    TreeTwistColumns,
};

const CLOSED_LOOP_MAX_ITERATIONS: usize = 50;
const CLOSED_LOOP_MAX_BACKTRACKS: usize = 32;
const CLOSED_LOOP_RESIDUAL_TOLERANCE: f64 = 1.0e-10;
const CLOSED_LOOP_STEP_TOLERANCE: f64 = 1.0e-12;

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
        // Reuse progress-capable implementation without reporting events.
        self.solve_at_with_progress(time, |_| {})
    }

    /// Solves the mechanism configuration at `time`, reporting progress
    /// through the callback.
    ///
    /// Uses the same initialization and state updates as [`Self::solve_at`].
    /// The callback receives closed-loop iteration events and a
    /// [`SolverProgress::Failed`] event if the solve returns an error.
    /// Successful tree-only solves emit no progress events.
    ///
    /// # Errors
    ///
    /// Returns an error if `time` is not finite or the closed-loop solve fails.
    /// A failed solve leaves the stored solution unchanged.
    pub fn solve_at_with_progress<F>(
        &mut self,
        time: f64,
        mut progress: F,
    ) -> Result<BodyPoses, SolverError>
    where
        F: FnMut(SolverProgress),
    {
        let mut candidates = self.candidates.clone();
        let result = solve_at_internal(
            self.problem,
            time,
            &mut candidates,
            self.has_previous_solution,
            &mut progress,
        );
        if result.is_ok() {
            self.candidates = candidates;
            self.has_previous_solution = true;
        } else {
            progress(SolverProgress::Failed);
        }
        result
    }
}

pub fn solve(problem: &Model) -> Result<BodyPoses, SolverError> {
    solve_at(problem, 0.0)
}
pub fn solve_at(problem: &Model, time: f64) -> Result<BodyPoses, SolverError> {
    SequenceSolver::new(problem).solve_at(time)
}
pub fn solve_at_with_progress<F>(
    problem: &Model,
    time: f64,
    mut progress: F,
) -> Result<BodyPoses, SolverError>
where
    F: FnMut(SolverProgress),
{
    let mut candidates = GeneralizedCoordinates::new(problem.coordinate_layout());
    let result = solve_at_internal(problem, time, &mut candidates, false, &mut progress);
    if result.is_err() {
        progress(SolverProgress::Failed);
    }
    result
}

fn solve_at_internal(
    problem: &Model,
    time: f64,
    candidates: &mut GeneralizedCoordinates,
    continuation: bool,
    progress: &mut dyn FnMut(SolverProgress),
) -> Result<BodyPoses, SolverError> {
    validate_time(time)?;
    if problem.closure_joint_ids().is_empty() {
        Ok(tree_poses_for_configuration_at(problem, candidates, time)
            .expect("solver configuration matches problem layout"))
    } else {
        solve_closed_loop(problem, time, candidates, continuation, progress)
    }
}

pub fn validate_time(time: f64) -> Result<(), SolverError> {
    if time.is_finite() {
        Ok(())
    } else {
        Err(SolverError::NonFiniteTime { time })
    }
}

fn solve_closed_loop(
    problem: &Model,
    time: f64,
    candidates: &mut GeneralizedCoordinates,
    continuation: bool,
    progress: &mut dyn FnMut(SolverProgress),
) -> Result<BodyPoses, SolverError> {
    for iteration in 0..CLOSED_LOOP_MAX_ITERATIONS {
        let poses = tree_poses_for_configuration_at(problem, candidates, time)
            .expect("solver configuration matches problem layout");
        let residuals = closure_residuals_at(problem, &poses, time);
        let residual_norm = DVector::from_vec(residuals.clone()).norm();
        if !residual_norm.is_finite() {
            return Err(SolverError::NonFiniteResidual);
        }
        progress(SolverProgress::Residual {
            iteration: iteration + 1,
            residual_norm,
        });
        if residual_norm <= CLOSED_LOOP_RESIDUAL_TOLERANCE {
            progress(SolverProgress::Converged {
                iterations: iteration,
                residual_norm,
            });
            return Ok(poses);
        }
        let analysis = analyze_closure_jacobian_for_configuration_at(problem, candidates, time)?;
        let free_coordinate_count = problem.free_primary_coordinates().len();
        if !continuation && free_coordinate_count > analysis.selected_columns().len() {
            return Err(SolverError::UnderDetermined {
                free_coordinates: free_coordinate_count,
                dependent_coordinates: analysis.selected_columns().len(),
                remaining_dofs: free_coordinate_count - analysis.selected_columns().len(),
            });
        }
        let selected = analysis.selected_columns();
        if selected.is_empty() {
            return Err(SolverError::NoIndependentCoordinates);
        }
        let jacobian = DMatrix::from_columns(
            &selected
                .iter()
                .map(|column| analysis.matrix().column(*column).into_owned())
                .collect::<Vec<_>>(),
        );
        let step = jacobian
            .svd(true, true)
            .solve(&(-DVector::from_vec(residuals)), analysis.tolerance())
            .map_err(SolverError::LinearSolveFailed)?;
        if step.iter().any(|value| !value.is_finite()) {
            return Err(SolverError::NonFiniteStep);
        }
        let mut accepted_candidates = None;
        let mut accepted_step_norm = 0.0;
        for attempt in 0..CLOSED_LOOP_MAX_BACKTRACKS {
            let scale = 0.5_f64.powi(attempt as i32);
            let mut trial_candidates = candidates.clone();
            for (index, column) in selected.iter().enumerate() {
                trial_candidates
                    .add_primary(*column, scale * step[index])
                    .expect("closure Jacobian columns belong to configuration");
            }
            let trial_poses = tree_poses_for_configuration_at(problem, &trial_candidates, time)
                .expect("solver configuration matches problem layout");
            let trial_residual_norm =
                DVector::from_vec(closure_residuals_at(problem, &trial_poses, time)).norm();
            if trial_residual_norm.is_finite() && trial_residual_norm < residual_norm {
                accepted_step_norm = scale * step.norm();
                accepted_candidates = Some(trial_candidates);
                break;
            }
        }
        let Some(updated_candidates) = accepted_candidates else {
            return Err(SolverError::NonConvergent {
                iterations: iteration + 1,
                residual_norm,
            });
        };
        progress(SolverProgress::StepAccepted {
            iteration: iteration + 1,
            residual_norm,
            step_norm: accepted_step_norm,
            damping_factor: accepted_step_norm / step.norm(),
            selected_rank: analysis.rank(),
        });
        *candidates = updated_candidates;
        if accepted_step_norm <= CLOSED_LOOP_STEP_TOLERANCE {
            let updated_poses = tree_poses_for_configuration_at(problem, candidates, time)
                .expect("solver configuration matches problem layout");
            let updated_residual_norm =
                DVector::from_vec(closure_residuals_at(problem, &updated_poses, time)).norm();
            if !updated_residual_norm.is_finite() {
                return Err(SolverError::NonFiniteResidual);
            }
            if updated_residual_norm <= CLOSED_LOOP_RESIDUAL_TOLERANCE {
                progress(SolverProgress::Converged {
                    iterations: iteration + 1,
                    residual_norm: updated_residual_norm,
                });
                return Ok(updated_poses);
            }
            return Err(SolverError::NonConvergent {
                iterations: iteration + 1,
                residual_norm: updated_residual_norm,
            });
        }
    }
    let poses = tree_poses_for_configuration_at(problem, candidates, time)
        .expect("solver configuration matches problem layout");
    let residual_norm = DVector::from_vec(closure_residuals_at(problem, &poses, time)).norm();
    if !residual_norm.is_finite() {
        return Err(SolverError::NonFiniteResidual);
    }
    Err(SolverError::NonConvergent {
        iterations: CLOSED_LOOP_MAX_ITERATIONS,
        residual_norm,
    })
}

#[derive(Debug)]
pub struct ClosureJacobian {
    matrix: DMatrix<f64>,
    columns: Vec<(JointId, usize)>,
    selected_columns: Vec<usize>,
    rank: usize,
    tolerance: f64,
}
impl ClosureJacobian {
    pub fn matrix(&self) -> &DMatrix<f64> {
        &self.matrix
    }
    pub fn columns(&self) -> &[(JointId, usize)] {
        &self.columns
    }
    pub fn selected_columns(&self) -> &[usize] {
        &self.selected_columns
    }
    pub fn selected_coordinates(&self) -> Vec<(JointId, usize)> {
        self.selected_columns
            .iter()
            .map(|column| self.columns[*column])
            .collect()
    }
    pub fn rank(&self) -> usize {
        self.rank
    }
    pub fn tolerance(&self) -> f64 {
        self.tolerance
    }
    pub fn residual_dimension(&self) -> usize {
        self.matrix.nrows()
    }
}

pub fn analyze_closure_jacobian(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> Result<ClosureJacobian, JacobianError> {
    analyze_closure_jacobian_at(problem, candidates, 0.0)
}
pub fn analyze_closure_jacobian_at(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> Result<ClosureJacobian, JacobianError> {
    analyze_closure_jacobian_for_configuration_at(
        problem,
        &GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), candidates),
        time,
    )
}
pub fn analyze_closure_jacobian_for_configuration(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
) -> Result<ClosureJacobian, JacobianError> {
    analyze_closure_jacobian_for_configuration_at(problem, configuration, 0.0)
}
pub fn analyze_closure_jacobian_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<ClosureJacobian, JacobianError> {
    let columns = problem.primary_coordinates();
    let free_columns = problem.free_primary_coordinates();
    let matrix =
        closure_jacobian_for_columns_at(problem, configuration, columns.clone(), true, time)?;
    let free_indices = columns
        .iter()
        .enumerate()
        .filter_map(|(index, column)| free_columns.contains(column).then_some(index))
        .collect::<Vec<_>>();
    let free_matrix = if free_indices.is_empty() {
        DMatrix::zeros(matrix.nrows(), 0)
    } else {
        DMatrix::from_columns(
            &free_indices
                .iter()
                .map(|index| matrix.column(*index).into_owned())
                .collect::<Vec<_>>(),
        )
    };
    if matrix
        .iter()
        .chain(free_matrix.iter())
        .any(|value| !value.is_finite())
    {
        return Err(JacobianError::NonFinite);
    }
    let free_singular_values = if free_matrix.nrows() == 0 || free_matrix.ncols() == 0 {
        Vec::new()
    } else {
        free_matrix
            .clone()
            .svd(false, false)
            .singular_values
            .as_slice()
            .to_vec()
    };
    let largest_free_singular_value = free_singular_values.iter().copied().fold(0.0, f64::max);
    let tolerance = 1.0e-12
        * free_matrix.nrows().max(free_matrix.ncols()).max(1) as f64
        * largest_free_singular_value;
    let rank = if free_matrix.nrows() == 0 || free_matrix.ncols() == 0 {
        0
    } else {
        free_matrix.clone().svd(false, false).rank(tolerance)
    };
    let mut selected_columns = Vec::new();
    for (free_column, column) in free_indices.iter().enumerate() {
        let mut selected = selected_columns
            .iter()
            .map(|index| matrix.column(*index).into_owned())
            .collect::<Vec<_>>();
        selected.push(free_matrix.column(free_column).into_owned());
        if DMatrix::from_columns(&selected)
            .svd(false, false)
            .rank(tolerance)
            > selected_columns.len()
        {
            selected_columns.push(*column);
        }
    }
    let poses = tree_poses_for_configuration_at(problem, configuration, time)
        .expect("solver configuration matches problem layout");
    let residuals = closure_residuals_at(problem, &poses, time);
    if selected_columns.len() != rank
        || (rank == 0
            && residuals
                .iter()
                .any(|value| !value.is_finite() || value.abs() > tolerance))
    {
        return Err(JacobianError::InsufficientCandidateRank {
            rank,
            selected: selected_columns.len(),
        });
    }
    Ok(ClosureJacobian {
        matrix,
        columns,
        selected_columns,
        rank,
        tolerance,
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SolverProgress {
    Residual {
        iteration: usize,
        residual_norm: f64,
    },
    StepAccepted {
        iteration: usize,
        residual_norm: f64,
        step_norm: f64,
        damping_factor: f64,
        selected_rank: usize,
    },
    Converged {
        iterations: usize,
        residual_norm: f64,
    },
    Failed,
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
#[derive(Debug, Error)]
pub enum JacobianError {
    #[error(transparent)]
    Residual(#[from] ResidualError),
    #[error("closure Jacobian contains non-finite values")]
    NonFinite,
    #[error("closure Jacobian rank {rank} exceeds selected candidate rank {selected}")]
    InsufficientCandidateRank { rank: usize, selected: usize },
}

#[cfg(test)]
mod tests {
    use crate::io::yaml::parse_yaml_str;

    use super::*;

    #[test]
    fn sequence_solver_commits_candidates_only_after_success() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_three_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let mut solver = SequenceSolver::new(&problem);
        solver.solve_at(89.0).unwrap();
        let candidates = solver.candidates.clone();
        let mut residual_events = 0;

        let interrupted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = solver.solve_at_with_progress(89.5, |progress| {
                if matches!(progress, SolverProgress::Residual { .. }) {
                    residual_events += 1;
                    if residual_events == 2 {
                        panic!("interrupt solve after candidate update");
                    }
                }
            });
        }));

        assert!(interrupted.is_err());
        assert_eq!(residual_events, 2);
        assert_eq!(solver.candidates, candidates);
    }

    #[test]
    fn reports_deterministic_closure_jacobian_rank_and_selection() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let analysis = analyze_closure_jacobian(&problem, &BTreeMap::new()).unwrap();

        assert_eq!(analysis.residual_dimension(), 3);
        assert_eq!(analysis.rank(), 2);
        assert_eq!(analysis.selected_columns(), &[0, 1]);
        assert!(analysis.tolerance() > 0.0);
        assert_eq!(analysis.selected_coordinates().len(), 2);
    }
}
