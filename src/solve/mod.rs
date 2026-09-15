use std::collections::BTreeMap;

use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};
use thiserror::Error;

use crate::model::{BodyId, JointId, Marker};
use crate::problem::{PreparedProblem, tree::TraversalDirection};

pub struct BodyPoses {
    poses: BTreeMap<BodyId, BodyPose>,
}

const CLOSED_LOOP_MAX_ITERATIONS: usize = 50;
const CLOSED_LOOP_MAX_BACKTRACKS: usize = 32;
const CLOSED_LOOP_RESIDUAL_TOLERANCE: f64 = 1.0e-10;
const CLOSED_LOOP_STEP_TOLERANCE: f64 = 1.0e-12;

impl BodyPoses {
    pub fn get(&self, body_id: BodyId) -> Option<&BodyPose> {
        self.poses.get(&body_id)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&BodyId, &BodyPose)> {
        self.poses.iter()
    }
}

pub fn solve(problem: &PreparedProblem) -> Result<BodyPoses, SolverError> {
    solve_at(problem, 0.0)
}

pub fn solve_at(problem: &PreparedProblem, time: f64) -> Result<BodyPoses, SolverError> {
    SequenceSolver::new(problem).solve_at(time)
}

pub struct SequenceSolver<'a> {
    problem: &'a PreparedProblem,
    candidates: BTreeMap<JointId, Vector3<f64>>,
    has_previous_solution: bool,
}

impl<'a> SequenceSolver<'a> {
    pub fn new(problem: &'a PreparedProblem) -> Self {
        Self {
            problem,
            candidates: BTreeMap::new(),
            has_previous_solution: false,
        }
    }

    pub fn solve_at(&mut self, time: f64) -> Result<BodyPoses, SolverError> {
        self.solve_at_with_progress(time, |_| {})
    }

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

pub fn solve_at_with_progress<F>(
    problem: &PreparedProblem,
    time: f64,
    mut progress: F,
) -> Result<BodyPoses, SolverError>
where
    F: FnMut(SolverProgress),
{
    let mut candidates = BTreeMap::new();
    let result = solve_at_internal(problem, time, &mut candidates, false, &mut progress);
    if result.is_err() {
        progress(SolverProgress::Failed);
    }
    result
}

fn solve_at_internal(
    problem: &PreparedProblem,
    time: f64,
    candidates: &mut BTreeMap<JointId, Vector3<f64>>,
    continuation: bool,
    progress: &mut dyn FnMut(SolverProgress),
) -> Result<BodyPoses, SolverError> {
    validate_time(time)?;

    if !problem.closure_joint_ids().is_empty() {
        return solve_closed_loop(problem, time, candidates, continuation, progress);
    }

    Ok(tree_poses_at(problem, time))
}

pub fn validate_time(time: f64) -> Result<(), SolverError> {
    if time.is_finite() {
        Ok(())
    } else {
        Err(SolverError::NonFiniteTime { time })
    }
}

fn solve_closed_loop(
    problem: &PreparedProblem,
    time: f64,
    candidates: &mut BTreeMap<JointId, Vector3<f64>>,
    continuation: bool,
    progress: &mut dyn FnMut(SolverProgress),
) -> Result<BodyPoses, SolverError> {
    for iteration in 0..CLOSED_LOOP_MAX_ITERATIONS {
        let poses = tree_poses_for_candidates_at(problem, candidates, time);
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
                let (joint_id, component) = analysis.columns()[*column];
                trial_candidates
                    .entry(joint_id)
                    .or_insert_with(Vector3::zeros)[component] += scale * step[index];
            }
            let trial_poses = tree_poses_for_candidates_at(problem, &trial_candidates, time);
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
            let updated_poses = tree_poses_for_candidates_at(problem, candidates, time);
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

    let poses = tree_poses_for_candidates_at(problem, candidates, time);
    let residual_norm = DVector::from_vec(closure_residuals_at(problem, &poses, time)).norm();
    if !residual_norm.is_finite() {
        return Err(SolverError::NonFiniteResidual);
    }
    Err(SolverError::NonConvergent {
        iterations: CLOSED_LOOP_MAX_ITERATIONS,
        residual_norm,
    })
}

pub fn tree_poses(problem: &PreparedProblem) -> BodyPoses {
    tree_poses_at(problem, 0.0)
}

pub fn tree_poses_at(problem: &PreparedProblem, time: f64) -> BodyPoses {
    tree_poses_for_candidates_at(problem, &BTreeMap::new(), time)
}

pub fn tree_poses_for_candidates(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> BodyPoses {
    tree_poses_for_candidates_at(problem, candidates, 0.0)
}

pub fn tree_poses_for_candidates_at(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> BodyPoses {
    let mut poses = BTreeMap::new();

    poses.insert(
        BodyId::GROUND,
        BodyPose::new(Vector3::zeros(), UnitQuaternion::identity()),
    );

    for edge in problem.tree_edges() {
        let joint = problem
            .model()
            .joints()
            .get(edge.joint_id())
            .expect("PreparedProblem contains missing joint");

        let parent = poses
            .get(&edge.parent_body_id())
            .expect("PreparedProblem contains unsolved parent");

        let (parent_marker, child_marker) = match edge.direction() {
            TraversalDirection::IToJ => (joint.i_marker(), joint.j_marker()),
            TraversalDirection::JToI => (joint.j_marker(), joint.i_marker()),
        };
        let child = spherical_child_pose(
            parent,
            parent_marker,
            child_marker,
            edge.traversal_relative_orientation_at(
                time,
                candidates
                    .get(&edge.joint_id())
                    .copied()
                    .unwrap_or_else(Vector3::zeros),
            ),
        );

        poses.insert(edge.child_body_id(), child);
    }

    BodyPoses { poses }
}

pub type TreeTwistColumns = (Vec<(JointId, usize)>, BTreeMap<BodyId, Vec<BodyTwist>>);

pub fn tree_twist_columns(problem: &PreparedProblem) -> TreeTwistColumns {
    tree_twist_columns_for_candidates(problem, &BTreeMap::new())
}

pub fn tree_twist_columns_for_candidates(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> TreeTwistColumns {
    tree_twist_columns_for_candidates_at(problem, candidates, 0.0)
}

pub fn tree_twist_columns_for_candidates_at(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> TreeTwistColumns {
    let columns = problem.free_primary_coordinates();
    tree_twist_columns_for_columns_at(problem, candidates, columns, false, time)
}

fn tree_twist_columns_for_columns_at(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    columns: Vec<(JointId, usize)>,
    include_prescribed: bool,
    time: f64,
) -> TreeTwistColumns {
    let poses = tree_poses_for_candidates_at(problem, candidates, time);
    let zero_twist = BodyTwist::new(Vector3::zeros(), Vector3::zeros());
    let mut twists = BTreeMap::from([(BodyId::GROUND, vec![zero_twist; columns.len()])]);

    for edge in problem.tree_edges() {
        let joint = problem
            .model()
            .joints()
            .get(edge.joint_id())
            .expect("PreparedProblem contains missing joint");
        let parent_pose = poses
            .get(edge.parent_body_id())
            .expect("PreparedProblem contains unsolved parent");
        let child_pose = poses
            .get(edge.child_body_id())
            .expect("PreparedProblem contains unsolved child");
        let parent_twists = twists
            .get(&edge.parent_body_id())
            .expect("PreparedProblem contains missing parent twists");
        let (parent_marker, child_marker) = match edge.direction() {
            TraversalDirection::IToJ => (joint.i_marker(), joint.j_marker()),
            TraversalDirection::JToI => (joint.j_marker(), joint.i_marker()),
        };
        let parent_marker_orientation = parent_pose.marker_orientation(parent_marker);
        let parent_marker_offset = parent_pose
            .orientation()
            .transform_vector(&parent_marker.position());
        let child_marker_offset = child_pose
            .orientation()
            .transform_vector(&child_marker.position());
        let displacement = edge.joint_coordinate().resolve_displacement_at(
            time,
            candidates
                .get(&edge.joint_id())
                .copied()
                .unwrap_or_else(Vector3::zeros),
        );
        let mut child_twists = Vec::with_capacity(columns.len());

        for (column, parent_twist) in parent_twists.iter().enumerate() {
            let mut displacement_rate = Vector3::zeros();
            if columns[column].0 == edge.joint_id()
                && (include_prescribed
                    || edge
                        .joint_coordinate()
                        .free_component_indices()
                        .contains(&columns[column].1))
            {
                displacement_rate[columns[column].1] = 1.0;
            }
            let relative_angular_velocity =
                edge.traversal_relative_angular_velocity(displacement, displacement_rate);
            let parent_marker_velocity = parent_twist.linear_velocity
                + parent_twist.angular_velocity.cross(&parent_marker_offset);
            let child_angular_velocity = parent_twist.angular_velocity
                + parent_marker_orientation.transform_vector(&relative_angular_velocity);
            let child_linear_velocity =
                parent_marker_velocity - child_angular_velocity.cross(&child_marker_offset);

            child_twists.push(BodyTwist::new(
                child_linear_velocity,
                child_angular_velocity,
            ));
        }

        twists.insert(edge.child_body_id(), child_twists);
    }

    (columns, twists)
}

pub fn closure_jacobian(problem: &PreparedProblem) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_candidates(problem, &BTreeMap::new())
}

pub fn closure_jacobian_for_candidates(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_candidates_at(problem, candidates, 0.0)
}

pub fn closure_jacobian_for_candidates_at(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> Result<DMatrix<f64>, ResidualError> {
    let columns = problem.free_primary_coordinates();
    closure_jacobian_for_columns_at(problem, candidates, columns, false, time)
}

fn closure_jacobian_for_columns_at(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    columns: Vec<(JointId, usize)>,
    include_prescribed: bool,
    time: f64,
) -> Result<DMatrix<f64>, ResidualError> {
    let poses = tree_poses_for_candidates_at(problem, candidates, time);
    let (_, body_twist_columns) = tree_twist_columns_for_columns_at(
        problem,
        candidates,
        columns.clone(),
        include_prescribed,
        time,
    );
    let mut column_rates = Vec::with_capacity(columns.len());

    for column in 0..columns.len() {
        let twists = body_twist_columns
            .iter()
            .map(|(body_id, body_twists)| (*body_id, body_twists[column]))
            .collect();
        column_rates.push(closure_residual_rates(problem, &poses, &twists)?);
    }

    let row_count = column_rates.first().map_or_else(
        || closure_residuals_at(problem, &poses, time).len(),
        Vec::len,
    );
    let mut jacobian = DMatrix::zeros(row_count, columns.len());

    for (column, rates) in column_rates.into_iter().enumerate() {
        for (row, rate) in rates.into_iter().enumerate() {
            jacobian[(row, column)] = rate;
        }
    }

    Ok(jacobian)
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
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> Result<ClosureJacobian, JacobianError> {
    analyze_closure_jacobian_at(problem, candidates, 0.0)
}

pub fn analyze_closure_jacobian_at(
    problem: &PreparedProblem,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> Result<ClosureJacobian, JacobianError> {
    let columns = problem.primary_coordinates();
    let free_columns = problem.free_primary_coordinates();
    let matrix = closure_jacobian_for_columns_at(problem, candidates, columns.clone(), true, time)?;
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
        let candidate_matrix = DMatrix::from_columns(&selected);

        if candidate_matrix.clone().svd(false, false).rank(tolerance) > selected_columns.len() {
            selected_columns.push(*column);
        }
    }

    let residuals = closure_residuals_at(
        problem,
        &tree_poses_for_candidates_at(problem, candidates, time),
        time,
    );
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

pub fn closure_position_residuals(
    problem: &PreparedProblem,
    poses: &BodyPoses,
) -> Vec<Vector3<f64>> {
    problem
        .closure_joint_ids()
        .iter()
        .map(|joint_id| {
            let joint = problem
                .model()
                .joints()
                .get(*joint_id)
                .expect("PreparedProblem contains missing closure joint");

            let i_pose = poses
                .get(joint.i_marker().body_id())
                .expect("closure joint i body has no pose");

            let j_pose = poses
                .get(joint.j_marker().body_id())
                .expect("closure joint j body has no pose");

            i_pose.marker_position(joint.i_marker()) - j_pose.marker_position(joint.j_marker())
        })
        .collect()
}

pub fn closure_orientation_residuals(problem: &PreparedProblem, poses: &BodyPoses) -> Vec<f64> {
    closure_orientation_residuals_at(problem, poses, 0.0)
}

fn closure_orientation_residuals_at(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    time: f64,
) -> Vec<f64> {
    problem
        .closure_joint_ids()
        .iter()
        .flat_map(|joint_id| {
            let joint = problem
                .model()
                .joints()
                .get(*joint_id)
                .expect("PreparedProblem contains missing closure joint");
            let coordinate = problem
                .joint_coordinate(*joint_id)
                .expect("PreparedProblem contains missing joint coordinate");
            let i_pose = poses
                .get(joint.i_marker().body_id())
                .expect("closure joint i body has no pose");
            let j_pose = poses
                .get(joint.j_marker().body_id())
                .expect("closure joint j body has no pose");
            let actual = i_pose.marker_orientation(joint.i_marker()).inverse()
                * j_pose.marker_orientation(joint.j_marker());
            let actual_displacement =
                (actual * coordinate.reference_orientation().inverse()).scaled_axis();
            let requested_displacement = coordinate.displacement().at(time);
            let requested = requested_displacement.rotation();
            let canonical_requested = UnitQuaternion::from_scaled_axis(Vector3::new(
                requested.x.unwrap_or(0.0),
                requested.y.unwrap_or(0.0),
                requested.z.unwrap_or(0.0),
            ))
            .scaled_axis();

            [requested.x, requested.y, requested.z]
                .into_iter()
                .enumerate()
                .filter_map(move |(index, requested)| {
                    requested.map(|_| actual_displacement[index] - canonical_requested[index])
                })
        })
        .collect()
}

pub struct BodyPose {
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyTwist {
    linear_velocity: Vector3<f64>,
    angular_velocity: Vector3<f64>,
}

impl BodyTwist {
    pub fn new(linear_velocity: Vector3<f64>, angular_velocity: Vector3<f64>) -> Self {
        Self {
            linear_velocity,
            angular_velocity,
        }
    }

    pub fn linear_velocity(&self) -> Vector3<f64> {
        self.linear_velocity
    }

    pub fn angular_velocity(&self) -> Vector3<f64> {
        self.angular_velocity
    }
}

impl BodyPose {
    pub fn new(position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn marker_orientation(&self, marker: &Marker) -> UnitQuaternion<f64> {
        self.orientation() * marker.orientation()
    }

    pub fn marker_position(&self, marker: &Marker) -> Vector3<f64> {
        self.position() + self.orientation().transform_vector(&marker.position())
    }

    pub fn marker_velocity(&self, marker: &Marker, twist: BodyTwist) -> Vector3<f64> {
        twist.linear_velocity
            + twist
                .angular_velocity
                .cross(&self.orientation().transform_vector(&marker.position()))
    }
}

pub fn closure_position_residual_rates(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<Vector3<f64>>, ResidualError> {
    let mut rates = Vec::new();

    for joint_id in problem.closure_joint_ids() {
        let joint = problem
            .model()
            .joints()
            .get(*joint_id)
            .expect("PreparedProblem contains missing closure joint");
        let i_pose = poses
            .get(joint.i_marker().body_id())
            .expect("closure joint i body has no pose");
        let j_pose = poses
            .get(joint.j_marker().body_id())
            .expect("closure joint j body has no pose");
        let i_twist =
            *twists
                .get(&joint.i_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.i_marker().body_id(),
                })?;
        let j_twist =
            *twists
                .get(&joint.j_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.j_marker().body_id(),
                })?;

        rates.push(
            i_pose.marker_velocity(joint.i_marker(), i_twist)
                - j_pose.marker_velocity(joint.j_marker(), j_twist),
        );
    }

    Ok(rates)
}

pub fn closure_orientation_residual_rates(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<f64>, ResidualError> {
    let mut rates = Vec::new();

    for joint_id in problem.closure_joint_ids() {
        let joint = problem
            .model()
            .joints()
            .get(*joint_id)
            .expect("PreparedProblem contains missing closure joint");
        let coordinate = problem
            .joint_coordinate(*joint_id)
            .expect("PreparedProblem contains missing joint coordinate");
        let i_pose = poses
            .get(joint.i_marker().body_id())
            .expect("closure joint i body has no pose");
        let j_pose = poses
            .get(joint.j_marker().body_id())
            .expect("closure joint j body has no pose");
        let i_twist =
            *twists
                .get(&joint.i_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.i_marker().body_id(),
                })?;
        let j_twist =
            *twists
                .get(&joint.j_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.j_marker().body_id(),
                })?;
        let actual = i_pose.marker_orientation(joint.i_marker()).inverse()
            * j_pose.marker_orientation(joint.j_marker());
        let actual_displacement =
            (actual * coordinate.reference_orientation().inverse()).scaled_axis();
        let relative_angular_velocity = i_pose
            .marker_orientation(joint.i_marker())
            .inverse_transform_vector(&(j_twist.angular_velocity - i_twist.angular_velocity));
        let actual_displacement_rate = coordinate.displacement_rate_from_relative_angular_velocity(
            actual_displacement,
            relative_angular_velocity,
        );
        let requested = coordinate.displacement().rotation();

        for (index, requested) in [requested.x, requested.y, requested.z]
            .into_iter()
            .enumerate()
        {
            if requested.is_some() {
                rates.push(actual_displacement_rate[index]);
            }
        }
    }

    Ok(rates)
}

pub fn closure_residuals(problem: &PreparedProblem, poses: &BodyPoses) -> Vec<f64> {
    closure_residuals_at(problem, poses, 0.0)
}

fn closure_residuals_at(problem: &PreparedProblem, poses: &BodyPoses, time: f64) -> Vec<f64> {
    let position_rows = closure_position_residuals(problem, poses)
        .into_iter()
        .flat_map(|residual| [residual.x, residual.y, residual.z]);

    position_rows
        .chain(closure_orientation_residuals_at(problem, poses, time))
        .collect()
}

pub fn closure_residual_rates(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<f64>, ResidualError> {
    let position_rows = closure_position_residual_rates(problem, poses, twists)?
        .into_iter()
        .flat_map(|rate| [rate.x, rate.y, rate.z]);

    Ok(position_rows
        .chain(closure_orientation_residual_rates(problem, poses, twists)?)
        .collect())
}

pub fn spherical_child_pose(
    parent: &BodyPose,
    i_marker: &Marker,
    j_marker: &Marker,
    relative_orientation: UnitQuaternion<f64>,
) -> BodyPose {
    // R_child = R_parent A_i Q A_j⁻¹
    let child_orientation = parent.marker_orientation(i_marker)
        * relative_orientation
        * j_marker.orientation().inverse();

    // r_child = p_i_world − R_child s_j
    let child_position =
        parent.marker_position(i_marker) - child_orientation.transform_vector(&j_marker.position());

    BodyPose::new(child_position, child_orientation)
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
pub enum ResidualError {
    #[error("closure joint `{joint_id:?}` body `{body_id:?}` has no twist")]
    MissingBodyTwist { joint_id: JointId, body_id: BodyId },
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
    use super::*;
    use crate::io::yaml::parse_yaml_str;
    use crate::model::BodyId;
    use crate::problem::prepare;

    const TOLERANCE: f64 = 1.0e-12;

    #[test]
    fn sequence_solver_commits_candidates_only_after_success() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_three_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
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
    fn identity_markers_preserve_parent_pose() {
        let parent = parent_pose();
        let i_marker = marker(BodyId::GROUND, Vector3::zeros(), UnitQuaternion::identity());
        let j_marker = marker(BodyId::new(1), Vector3::zeros(), UnitQuaternion::identity());

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, UnitQuaternion::identity());

        assert_position_close(child.position(), parent.position());
        assert_orientation_close(child.orientation(), parent.orientation());
    }

    #[test]
    fn computes_child_pose_from_relative_orientation() {
        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let parent = parent_pose();
        let i_marker = marker(BodyId::GROUND, Vector3::zeros(), UnitQuaternion::identity());
        let j_marker = marker(BodyId::new(1), Vector3::zeros(), UnitQuaternion::identity());

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        assert_position_close(child.position(), parent.position());
        assert_orientation_close(
            child.orientation(),
            parent.orientation() * relative_orientation,
        );
    }

    #[test]
    fn combines_non_aligned_marker_frames_with_relative_orientation() {
        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let parent = parent_pose();

        let i_marker = marker(
            BodyId::GROUND,
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.2, -0.1, 0.3),
        );
        let j_marker = marker(
            BodyId::new(1),
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(-0.4, 0.5, -0.2),
        );

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        let expected = parent.marker_orientation(&i_marker) * relative_orientation;
        let actual = child.marker_orientation(&j_marker);

        assert_position_close(child.position(), parent.position());
        assert_orientation_close(actual, expected);
    }

    #[test]
    fn handles_nonzero_marker_offsets() {
        let parent = BodyPose::new(Vector3::new(1.0, 2.0, 3.0), UnitQuaternion::identity());
        let i_marker = marker(
            BodyId::GROUND,
            Vector3::new(2.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let j_marker = marker(
            BodyId::new(1),
            Vector3::new(0.0, 1.0, 0.0),
            UnitQuaternion::identity(),
        );

        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        let i_world = parent.marker_position(&i_marker);
        let j_world = child.marker_position(&j_marker);

        assert_position_close(i_world, j_world);
        assert_orientation_close(child.orientation(), relative_orientation);
    }

    #[test]
    fn evaluates_closed_loop_position_residuals_from_tree_poses() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let residuals = closure_position_residuals(&problem, &poses);

        assert_eq!(residuals.len(), 1);
        assert!(residuals[0].norm() < TOLERANCE);
    }

    #[test]
    fn evaluates_prescribed_closed_loop_orientation_residuals() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let residuals = closure_orientation_residuals(&problem, &poses);

        assert_eq!(residuals.len(), 1);
        assert!(residuals[0].abs() < TOLERANCE);
    }

    #[test]
    fn evaluates_closed_loop_position_residual_rates_from_body_twists() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let twists = BTreeMap::from([
            (
                BodyId::GROUND,
                BodyTwist::new(Vector3::zeros(), Vector3::zeros()),
            ),
            (
                BodyId::new(1),
                BodyTwist::new(Vector3::zeros(), Vector3::x()),
            ),
        ]);
        let rates = closure_position_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(rates.len(), 1);
        assert!(rates[0].norm() > 1.0);
    }

    #[test]
    fn rejects_missing_body_twist_for_closure_rates() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);

        assert!(matches!(
            closure_position_residual_rates(&problem, &poses, &BTreeMap::new()),
            Err(ResidualError::MissingBodyTwist { .. })
        ));
    }

    #[test]
    fn propagates_free_primary_twist_columns_in_deterministic_order() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_parse.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let (columns, twists) = tree_twist_columns(&problem);
        let body_twists = twists.get(&BodyId::new(1)).unwrap();

        assert_eq!(
            columns,
            vec![
                (crate::model::JointId::new(1), 0),
                (crate::model::JointId::new(1), 1),
                (crate::model::JointId::new(1), 2)
            ]
        );
        assert_eq!(body_twists.len(), 3);
        assert_eq!(body_twists[2].angular_velocity, Vector3::z());
        assert_eq!(body_twists[2].linear_velocity, Vector3::zeros());
    }

    #[test]
    fn preserves_non_contiguous_free_primary_columns() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
            .replace("      rot_z: 90", "      rot_x: 10\n      rot_z: 90");
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let (columns, twists) = tree_twist_columns(&problem);
        let body_twists = twists.get(&BodyId::new(1)).unwrap();

        assert_eq!(columns, vec![(crate::model::JointId::new(1), 1)]);
        assert!(body_twists[0].angular_velocity.norm() > 0.0);
    }

    #[test]
    fn validates_reversed_tree_twist_columns_against_pose_perturbations() {
        let input = parse_yaml_str(
            r#"hardpoints:
  P1: [0.0, 0.0, 0.0]
bodies:
  B1:
    body_id: 1
    side: single
    position: [0.0, 0.0, 0.0]
    orientation:
      method: euler
      euler_angles: [0, 0, 0]
    points_on_body: [P1]
joints:
  J1:
    joint_id: 1
    kind: spherical
    role: primary
    i:
      body_id: 1
      position: P1
      orientation:
        method: euler
        euler_angles: [0.2, -0.1, 0.3]
    j:
      body_id: 0
      position: P1
      orientation:
        method: euler
        euler_angles: [-0.4, 0.5, -0.2]
"#,
        )
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let candidates = BTreeMap::from([(crate::model::JointId::new(1), Vector3::zeros())]);
        let (columns, twists) = tree_twist_columns_for_candidates(&problem, &candidates);
        let step = 1.0e-7;

        for (column, (_, component)) in columns.iter().enumerate() {
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&crate::model::JointId::new(1)).unwrap()[*component] += step;
            backward.get_mut(&crate::model::JointId::new(1)).unwrap()[*component] -= step;
            let forward_orientation = tree_poses_for_candidates(&problem, &forward)
                .get(BodyId::new(1))
                .unwrap()
                .orientation();
            let backward_orientation = tree_poses_for_candidates(&problem, &backward)
                .get(BodyId::new(1))
                .unwrap()
                .orientation();
            let finite_difference =
                (forward_orientation * backward_orientation.inverse()).scaled_axis() / (2.0 * step);

            assert!(
                (twists[&BodyId::new(1)][column].angular_velocity - finite_difference).norm()
                    < 1.0e-6
            );
        }
    }

    #[test]
    fn assembles_closure_jacobian_from_twist_columns() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let jacobian = closure_jacobian(&problem).unwrap();

        assert_eq!(jacobian.nrows(), 3);
        assert_eq!(jacobian.ncols(), 2);
        assert!(jacobian.iter().any(|value| value.abs() > 1.0));
    }

    #[test]
    fn analytic_closure_jacobian_matches_candidate_perturbations() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let candidates =
            BTreeMap::from([(crate::model::JointId::new(1), Vector3::new(0.2, 0.1, 0.0))]);
        let jacobian = closure_jacobian_for_candidates(&problem, &candidates).unwrap();
        let step = 1.0e-7;

        for column in 0..jacobian.ncols() {
            let component = problem.free_primary_coordinates()[column].1;
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&crate::model::JointId::new(1)).unwrap()[component] += step;
            backward.get_mut(&crate::model::JointId::new(1)).unwrap()[component] -= step;
            let forward_residuals =
                closure_residuals(&problem, &tree_poses_for_candidates(&problem, &forward));
            let backward_residuals =
                closure_residuals(&problem, &tree_poses_for_candidates(&problem, &backward));

            for row in 0..jacobian.nrows() {
                let finite_difference =
                    (forward_residuals[row] - backward_residuals[row]) / (2.0 * step);
                assert!((jacobian[(row, column)] - finite_difference).abs() < 1.0e-6);
            }
        }
    }

    #[test]
    fn reports_deterministic_closure_jacobian_rank_and_selection() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let analysis = analyze_closure_jacobian(&problem, &BTreeMap::new()).unwrap();

        assert_eq!(analysis.residual_dimension(), 3);
        assert_eq!(analysis.rank(), 2);
        assert_eq!(analysis.selected_columns(), &[0, 1]);
        assert!(analysis.tolerance() > 0.0);
        assert_eq!(analysis.selected_coordinates().len(), 2);
    }

    #[test]
    fn rejects_over_prescribed_closure_jacobian() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
            .replace(
                "      rot_z: 90",
                "      rot_x: 10\n      rot_y: 20\n      rot_z: 90",
            );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();

        assert!(matches!(
            analyze_closure_jacobian(&problem, &BTreeMap::new()),
            Err(JacobianError::InsufficientCandidateRank {
                rank: 0,
                selected: 0
            })
        ));
    }

    #[test]
    fn evaluates_prescribed_closed_loop_orientation_residual_rates() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let twists = BTreeMap::from([
            (
                BodyId::GROUND,
                BodyTwist::new(Vector3::zeros(), Vector3::zeros()),
            ),
            (
                BodyId::new(1),
                BodyTwist::new(Vector3::zeros(), Vector3::z()),
            ),
        ]);
        let rates = closure_orientation_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(rates.len(), 1);
        assert!((rates[0] + 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn orders_position_rows_before_orientation_rows() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let twists = BTreeMap::from([
            (
                BodyId::GROUND,
                BodyTwist::new(Vector3::zeros(), Vector3::zeros()),
            ),
            (
                BodyId::new(1),
                BodyTwist::new(Vector3::zeros(), Vector3::z()),
            ),
        ]);
        let residuals = closure_residuals(&problem, &poses);
        let rates = closure_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(residuals.len(), 4);
        assert!(residuals.iter().all(|residual| residual.abs() < TOLERANCE));
        assert_eq!(rates.len(), 4);
        assert!(rates[..3].iter().all(|rate| rate.abs() < TOLERANCE));
        assert!((rates[3] + 1.0).abs() < TOLERANCE);
    }

    fn parent_pose() -> BodyPose {
        BodyPose::new(
            Vector3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        )
    }

    fn marker(body_id: BodyId, position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Marker {
        Marker::new("marker", body_id, position, orientation)
    }

    fn assert_position_close(actual: Vector3<f64>, expected: Vector3<f64>) {
        assert!((actual - expected).norm() < TOLERANCE);
    }

    fn assert_orientation_close(actual: UnitQuaternion<f64>, expected: UnitQuaternion<f64>) {
        assert!(actual.angle_to(&expected) < TOLERANCE);
    }
}
