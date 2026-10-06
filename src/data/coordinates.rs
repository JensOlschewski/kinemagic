use std::collections::BTreeMap;

use nalgebra::{DVector, Vector3};
use thiserror::Error;

use crate::model::coordinates::CoordinateLayout;
use crate::model::mechanism::JointId;

#[derive(Debug, Clone, PartialEq)]
pub struct GeneralizedCoordinates {
    values: DVector<f64>,
    layout_id: usize,
    primary_coordinates: Vec<(JointId, usize)>,
}

impl GeneralizedCoordinates {
    pub fn new(layout: &CoordinateLayout) -> Self {
        Self {
            values: DVector::zeros(layout.primary_coordinate_count()),
            layout_id: layout.id(),
            primary_coordinates: layout.primary_coordinates().to_vec(),
        }
    }

    pub fn from_values(
        layout: &CoordinateLayout,
        values: DVector<f64>,
    ) -> Result<Self, GeneralizedCoordinatesError> {
        if values.len() == layout.primary_coordinate_count() {
            Ok(Self {
                values,
                layout_id: layout.id(),
                primary_coordinates: layout.primary_coordinates().to_vec(),
            })
        } else {
            Err(GeneralizedCoordinatesError::DimensionMismatch {
                expected: layout.primary_coordinate_count(),
                actual: values.len(),
            })
        }
    }

    pub fn from_candidates(
        layout: &CoordinateLayout,
        candidates: &BTreeMap<JointId, Vector3<f64>>,
    ) -> Self {
        let mut q = Self::new(layout);
        for (joint_id, candidate) in candidates {
            for (column, (_, component)) in layout
                .primary_coordinates()
                .iter()
                .enumerate()
                .filter(|(_, (id, _))| id == joint_id)
            {
                q.values[column] = candidate[*component];
            }
        }
        q
    }

    pub fn values(&self) -> &DVector<f64> {
        &self.values
    }

    pub fn joint_values(&self, joint_id: JointId) -> Vector3<f64> {
        let mut values = Vector3::zeros();
        for (column, (id, component)) in self.primary_coordinates.iter().enumerate() {
            if *id == joint_id {
                values[*component] = self.values[column];
            }
        }
        values
    }

    pub fn validate_layout(
        &self,
        layout: &CoordinateLayout,
    ) -> Result<(), GeneralizedCoordinatesError> {
        if self.layout_id == layout.id() {
            Ok(())
        } else {
            Err(GeneralizedCoordinatesError::LayoutMismatch)
        }
    }

    pub fn set_primary(
        &mut self,
        column: usize,
        value: f64,
    ) -> Result<(), GeneralizedCoordinatesError> {
        let actual = self.values.len();
        let Some(current) = self.values.get_mut(column) else {
            return Err(GeneralizedCoordinatesError::PrimaryColumnOutOfRange { column, actual });
        };
        *current = value;
        Ok(())
    }

    pub fn add_primary(
        &mut self,
        column: usize,
        value: f64,
    ) -> Result<(), GeneralizedCoordinatesError> {
        let actual = self.values.len();
        let Some(current) = self.values.get_mut(column) else {
            return Err(GeneralizedCoordinatesError::PrimaryColumnOutOfRange { column, actual });
        };
        *current += value;
        Ok(())
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum GeneralizedCoordinatesError {
    #[error("generalized coordinates have {actual} values, expected {expected}")]
    DimensionMismatch { expected: usize, actual: usize },
    #[error("primary coordinate column {column} is out of range for {actual} values")]
    PrimaryColumnOutOfRange { column: usize, actual: usize },
    #[error("generalized coordinates belong to a different coordinate layout")]
    LayoutMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generalized_coordinates_reads_joint_values_and_updates_global_column() {
        let layout = CoordinateLayout::new(
            vec![
                (JointId::new(2), 0),
                (JointId::new(2), 1),
                (JointId::new(2), 2),
            ],
            vec![],
        );
        let mut q = GeneralizedCoordinates::new(&layout);

        q.set_primary(1, 0.5).unwrap();
        q.add_primary(1, 0.25).unwrap();

        assert_eq!(q.values().len(), 3);
        assert_eq!(
            q.joint_values(JointId::new(2)),
            Vector3::new(0.0, 0.75, 0.0)
        );
    }

    #[test]
    fn generalized_coordinates_uses_exact_interleaved_joint_columns() {
        let layout = CoordinateLayout::new(
            vec![
                (JointId::new(2), 0),
                (JointId::new(9), 1),
                (JointId::new(2), 2),
            ],
            vec![],
        );
        let q = GeneralizedCoordinates::from_candidates(
            &layout,
            &BTreeMap::from([
                (JointId::new(2), Vector3::new(1.0, 2.0, 3.0)),
                (JointId::new(9), Vector3::new(4.0, 5.0, 6.0)),
            ]),
        );

        assert_eq!(layout.primary_range(JointId::new(2)), None);
        assert_eq!(q.values().as_slice(), &[1.0, 5.0, 3.0]);
        assert_eq!(q.joint_values(JointId::new(2)), Vector3::new(1.0, 0.0, 3.0));
    }
}
