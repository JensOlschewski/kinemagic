use nalgebra::DMatrix;
use thiserror::Error;

use crate::data::coordinates::GeneralizedCoordinates;
use crate::model::Model;
use crate::model::mechanism::JointId;
use crate::solve::closure::{
    ResidualError, closure_jacobian_for_columns_at, closure_position_residuals,
    closure_residuals_at,
};
use crate::solve::tree::tree_poses_for_configuration_at;

/// Position residual norm above which a closure joint is treated as not
/// closed, rather than as floating-point noise around zero.
const INCOMPATIBILITY_TOLERANCE: f64 = 1.0e-8;

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

pub fn analyze_closure_jacobian_at(
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
    // With no residual rows (e.g. an open tree with no closure joints),
    // rank is trivially 0 and no column can raise it further: skip the
    // incremental selection below, which would otherwise SVD a 0-row
    // matrix.
    let mut selected_columns = Vec::new();
    if free_matrix.nrows() > 0 {
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

#[derive(Debug, Error)]
pub enum JacobianError {
    #[error(transparent)]
    Residual(#[from] ResidualError),
    #[error("closure Jacobian contains non-finite values")]
    NonFinite,
    #[error("closure Jacobian rank {rank} exceeds selected candidate rank {selected}")]
    InsufficientCandidateRank { rank: usize, selected: usize },
}

/// Classification of a model's mobility: free coordinates versus the
/// independent residual equations constraining them.
///
/// Not a closure-specific concept — it is a generic rank-versus-free-DOF
/// comparison. Today the only residual equations come from closure joints,
/// so an open tree only ever reaches `Determined` (both counts are zero);
/// nothing here assumes closure loops are the only possible source.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Mobility {
    /// Free coordinates and residual equations balance: rank equals both.
    Determined,
    /// Fewer independent residual equations than free coordinates.
    Underdetermined { mobility: usize },
    /// More residual equations than the Jacobian's rank accounts for.
    Overdetermined { redundant: usize },
}

impl std::fmt::Display for Mobility {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Determined => write!(f, "determined"),
            Self::Underdetermined { mobility } => {
                write!(f, "underdetermined (mobility {mobility})")
            }
            Self::Overdetermined { redundant } => {
                write!(f, "overdetermined (redundant {redundant})")
            }
        }
    }
}

#[derive(Debug, Error)]
pub enum MobilityError {
    #[error(transparent)]
    Jacobian(#[from] JacobianError),
    #[error("closure joint `{joint_id:?}` does not close: residual {residual:e} exceeds tolerance")]
    Incompatible { joint_id: JointId, residual: f64 },
}

/// Classifies mobility at `configuration` and `time`.
///
/// A closure joint whose position residual exceeds
/// [`INCOMPATIBILITY_TOLERANCE`] is reported first, as `Incompatible` — the
/// loop does not close at this configuration, regardless of how its
/// Jacobian ranks. Otherwise free coordinates are compared against the
/// closure Jacobian's rank and residual dimension.
pub fn mobility_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<Mobility, MobilityError> {
    let analysis = analyze_closure_jacobian_at(problem, configuration, time)?;

    let poses = tree_poses_for_configuration_at(problem, configuration, time)
        .expect("mobility_at configuration matches problem layout");
    let worst_offender = problem
        .closure_joint_ids()
        .iter()
        .zip(closure_position_residuals(problem, &poses))
        .map(|(joint_id, residual)| (*joint_id, residual.norm()))
        .max_by(|(_, left), (_, right)| left.total_cmp(right));
    if let Some((joint_id, residual)) = worst_offender
        && residual > INCOMPATIBILITY_TOLERANCE
    {
        return Err(MobilityError::Incompatible { joint_id, residual });
    }

    let free_coordinate_count = problem.free_primary_coordinates().len();
    let rank = analysis.rank();
    let residual_dimension = analysis.residual_dimension();

    Ok(if rank < free_coordinate_count {
        Mobility::Underdetermined {
            mobility: free_coordinate_count - rank,
        }
    } else if residual_dimension > rank {
        Mobility::Overdetermined {
            redundant: residual_dimension - rank,
        }
    } else {
        Mobility::Determined
    })
}

#[cfg(test)]
mod tests {
    use crate::io::yaml::parse_yaml_str;

    use super::*;

    #[test]
    fn reports_deterministic_closure_jacobian_rank_and_selection() {
        let problem = parse_yaml_str(include_str!(
            "../../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let configuration = GeneralizedCoordinates::new(problem.coordinate_layout());
        let analysis = analyze_closure_jacobian_at(&problem, &configuration, 0.0).unwrap();

        assert_eq!(analysis.residual_dimension(), 3);
        assert_eq!(analysis.rank(), 2);
        assert_eq!(analysis.selected_columns(), &[0, 1]);
        assert!(analysis.tolerance() > 0.0);
        assert_eq!(analysis.selected_coordinates().len(), 2);
    }

    fn mobility_for(fixture: &str) -> Result<Mobility, MobilityError> {
        let problem = parse_yaml_str(fixture).unwrap().into_model().unwrap();
        let configuration = GeneralizedCoordinates::new(problem.coordinate_layout());
        mobility_at(&problem, &configuration, 0.0)
    }

    #[test]
    fn open_tree_with_fully_prescribed_coordinates_is_determined() {
        let mobility = mobility_for(include_str!(
            "../../../tests/fixtures/revolute_one_body_motion.yaml"
        ))
        .unwrap();

        assert_eq!(mobility, Mobility::Determined);
    }

    #[test]
    fn redundant_closure_constraint_is_overdetermined() {
        // J1's rotation is fully prescribed (free count 2), while the
        // spherical-spherical closure leaves rank 2 but 3 residual rows:
        // one row is linearly dependent on the other two.
        let mobility = mobility_for(include_str!(
            "../../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap();

        assert_eq!(mobility, Mobility::Overdetermined { redundant: 1 });
    }

    #[test]
    fn undriven_closure_joint_rotation_is_underdetermined() {
        // Same loop, but J1 has no motion: the free spin about the P1-P2
        // axis that the closure can never constrain is now a leftover
        // degree of mobility instead of a redundant constraint row.
        let mobility = mobility_for(include_str!(
            "../../../tests/fixtures/spherical_closed_loop_underdetermined.yaml"
        ))
        .unwrap();

        assert_eq!(mobility, Mobility::Underdetermined { mobility: 1 });
    }

    #[test]
    fn loop_that_cannot_close_at_the_initial_configuration_is_incompatible() {
        let error = mobility_for(include_str!(
            "../../../tests/fixtures/closed_loop_incompatible.yaml"
        ))
        .unwrap_err();

        assert!(matches!(
            error,
            MobilityError::Incompatible { joint_id, residual }
                if joint_id == JointId::new(4) && residual > 1.0
        ));
    }
}
