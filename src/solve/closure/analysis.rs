use nalgebra::DMatrix;
use thiserror::Error;

use crate::data::coordinates::GeneralizedCoordinates;
use crate::model::Model;
use crate::model::mechanism::JointId;
use crate::solve::closure::{ResidualError, closure_jacobian_for_columns_at, closure_residuals_at};
use crate::solve::tree::tree_poses_for_configuration_at;

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
}
