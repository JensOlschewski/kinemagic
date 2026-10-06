use nalgebra::{DMatrix, DVector};

use crate::data::coordinates::GeneralizedCoordinates;
use crate::model::Model;
use crate::solve::closure::analysis::analyze_closure_jacobian_at;
use crate::solve::closure::closure_residuals_at;
use crate::solve::state::BodyPoses;
use crate::solve::tree::tree_poses_for_configuration_at;
use crate::solve::{SolverError, SolverProgress};

const CLOSED_LOOP_MAX_ITERATIONS: usize = 50;
const CLOSED_LOOP_MAX_BACKTRACKS: usize = 32;
const CLOSED_LOOP_RESIDUAL_TOLERANCE: f64 = 1.0e-10;
const CLOSED_LOOP_STEP_TOLERANCE: f64 = 1.0e-12;

pub(crate) fn solve_closed_loop(
    problem: &Model,
    time: f64,
    candidates: &mut GeneralizedCoordinates,
    continuation: bool,
    progress: &mut dyn FnMut(SolverProgress),
) -> Result<BodyPoses, SolverError> {
    for iteration in 0..CLOSED_LOOP_MAX_ITERATIONS {
        let (poses, residuals, residual_norm) = evaluate(problem, candidates, time);
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
        let analysis = analyze_closure_jacobian_at(problem, candidates, time)?;
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
            .solve(&(-residuals), analysis.tolerance())
            .map_err(SolverError::LinearSolveFailed)?;
        if step.iter().any(|value| !value.is_finite()) {
            return Err(SolverError::NonFiniteStep);
        }
        let accepted = backtrack(problem, candidates, time, selected, &step, residual_norm);
        let Some((updated_candidates, accepted_step_norm)) = accepted else {
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
            let (updated_poses, _, updated_residual_norm) = evaluate(problem, candidates, time);
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
    let (_, _, residual_norm) = evaluate(problem, candidates, time);
    if !residual_norm.is_finite() {
        return Err(SolverError::NonFiniteResidual);
    }
    Err(SolverError::NonConvergent {
        iterations: CLOSED_LOOP_MAX_ITERATIONS,
        residual_norm,
    })
}

fn evaluate(
    problem: &Model,
    candidates: &GeneralizedCoordinates,
    time: f64,
) -> (BodyPoses, DVector<f64>, f64) {
    let poses = tree_poses_for_configuration_at(problem, candidates, time)
        .expect("solver configuration matches problem layout");
    let residuals = DVector::from_vec(closure_residuals_at(problem, &poses, time));
    let norm = residuals.norm();
    (poses, residuals, norm)
}

/// Halves the Newton step until the residual norm decreases.
/// Returns the accepted coordinates and the scaled step norm.
fn backtrack(
    problem: &Model,
    candidates: &GeneralizedCoordinates,
    time: f64,
    selected: &[usize],
    step: &DVector<f64>,
    residual_norm: f64,
) -> Option<(GeneralizedCoordinates, f64)> {
    for attempt in 0..CLOSED_LOOP_MAX_BACKTRACKS {
        let scale = 0.5_f64.powi(attempt as i32);
        let mut trial = candidates.clone();
        for (index, column) in selected.iter().enumerate() {
            trial
                .add_primary(*column, scale * step[index])
                .expect("closure Jacobian columns belong to configuration");
        }
        let (_, _, trial_norm) = evaluate(problem, &trial, time);
        if trial_norm.is_finite() && trial_norm < residual_norm {
            return Some((trial, scale * step.norm()));
        }
    }
    None
}
