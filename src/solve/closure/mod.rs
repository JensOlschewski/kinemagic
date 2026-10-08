use std::collections::BTreeMap;

use nalgebra::{DMatrix, Matrix3, UnitQuaternion, Vector3};
use thiserror::Error;

use crate::data::coordinates::{GeneralizedCoordinates, GeneralizedCoordinatesError};
use crate::model::Model;
use crate::model::mechanism::joint::geometry;
use crate::model::mechanism::{BodyId, JointId};
use crate::solve::state::{BodyPoses, BodyTwist};
use crate::solve::tree::{tree_body_jacobians_for_columns_at, tree_poses_for_configuration_at};

pub mod analysis;
pub mod newton;

pub fn closure_jacobian_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_columns_at(
        problem,
        configuration,
        problem.free_primary_coordinates(),
        false,
        time,
    )
}
pub(crate) fn closure_jacobian_for_columns_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    columns: Vec<(JointId, usize)>,
    include_prescribed: bool,
    time: f64,
) -> Result<DMatrix<f64>, ResidualError> {
    configuration.validate_layout(problem.coordinate_layout())?;
    let poses = tree_poses_for_configuration_at(problem, configuration, time)?;
    let (_, body_jacobians) = tree_body_jacobians_for_columns_at(
        problem,
        configuration,
        columns,
        include_prescribed,
        time,
    )?;
    closure_jacobian_from_body_jacobians(problem, &poses, &body_jacobians, time)
}
fn closure_jacobian_from_body_jacobians(
    problem: &Model,
    poses: &BodyPoses,
    body_jacobians: &BTreeMap<BodyId, DMatrix<f64>>,
    time: f64,
) -> Result<DMatrix<f64>, ResidualError> {
    let column_count = body_jacobians
        .get(&BodyId::GROUND)
        .expect("Model contains missing ground Jacobian")
        .ncols();
    let position_row_count = 3 * problem.closure_joint_ids().len();
    let mut jacobian = DMatrix::zeros(
        closure_residuals_at(problem, poses, time).len(),
        column_count,
    );
    let mut orientation_row = position_row_count;
    for (closure_index, joint_id) in problem.closure_joint_ids().iter().enumerate() {
        let joint = problem
            .mechanism()
            .joints()
            .get(*joint_id)
            .expect("Model contains missing closure joint");
        let coordinate = problem
            .joint_coordinate(*joint_id)
            .expect("Model contains missing joint coordinate")
            .as_spherical()
            .expect("closure joints are validated to be spherical");
        let i_pose = poses
            .get(joint.i_marker().body_id())
            .expect("closure joint i body has no pose");
        let j_pose = poses
            .get(joint.j_marker().body_id())
            .expect("closure joint j body has no pose");
        let i_jacobian = body_jacobians.get(&joint.i_marker().body_id()).ok_or(
            ResidualError::MissingBodyTwist {
                joint_id: *joint_id,
                body_id: joint.i_marker().body_id(),
            },
        )?;
        let j_jacobian = body_jacobians.get(&joint.j_marker().body_id()).ok_or(
            ResidualError::MissingBodyTwist {
                joint_id: *joint_id,
                body_id: joint.j_marker().body_id(),
            },
        )?;
        let (i_block, j_block) =
            geometry::position_jacobian_blocks(i_pose, joint.i_marker(), j_pose, joint.j_marker());
        let position_block = i_block * i_jacobian.rows(0, 6) + j_block * j_jacobian.rows(0, 6);
        jacobian
            .rows_mut(3 * closure_index, 3)
            .copy_from(&position_block);
        let actual = i_pose.marker_orientation(joint.i_marker()).inverse()
            * j_pose.marker_orientation(joint.j_marker());
        let actual_displacement =
            (actual * coordinate.reference_orientation().inverse()).scaled_axis();
        let displacement_rate_matrix = Matrix3::from_columns(&[
            coordinate.displacement_rate_from_relative_angular_velocity(
                actual_displacement,
                Vector3::x(),
            ),
            coordinate.displacement_rate_from_relative_angular_velocity(
                actual_displacement,
                Vector3::y(),
            ),
            coordinate.displacement_rate_from_relative_angular_velocity(
                actual_displacement,
                Vector3::z(),
            ),
        ]);
        let relative_angular_velocity = i_pose
            .marker_orientation(joint.i_marker())
            .inverse()
            .to_rotation_matrix()
            .matrix()
            * (j_jacobian.fixed_rows::<3>(3).into_owned()
                - i_jacobian.fixed_rows::<3>(3).into_owned());
        let displacement_rate = displacement_rate_matrix * relative_angular_velocity;
        let requested = coordinate.displacement().rotation();
        for (component, requested) in [requested.x, requested.y, requested.z]
            .into_iter()
            .enumerate()
        {
            if requested.is_some() {
                jacobian
                    .row_mut(orientation_row)
                    .copy_from(&displacement_rate.row(component));
                orientation_row += 1;
            }
        }
    }
    Ok(jacobian)
}

pub fn closure_position_residuals(problem: &Model, poses: &BodyPoses) -> Vec<Vector3<f64>> {
    // Each spherical closure requires its two marker origins to coincide.
    problem
        .closure_joint_ids()
        .iter()
        .map(|joint_id| {
            let joint = problem
                .mechanism()
                .joints()
                .get(*joint_id)
                .expect("Model contains missing closure joint");
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
pub fn closure_orientation_residuals_at(problem: &Model, poses: &BodyPoses, time: f64) -> Vec<f64> {
    problem
        .closure_joint_ids()
        .iter()
        .flat_map(|joint_id| {
            let joint = problem
                .mechanism()
                .joints()
                .get(*joint_id)
                .expect("Model contains missing closure joint");
            let coordinate = problem
                .joint_coordinate(*joint_id)
                .expect("Model contains missing joint coordinate")
                .as_spherical()
                .expect("closure joints are validated to be spherical");
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
pub fn closure_residuals_at(problem: &Model, poses: &BodyPoses, time: f64) -> Vec<f64> {
    closure_position_residuals(problem, poses)
        .into_iter()
        .flat_map(|residual| [residual.x, residual.y, residual.z])
        .chain(closure_orientation_residuals_at(problem, poses, time))
        .collect()
}

pub fn closure_position_residual_rates(
    problem: &Model,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<Vector3<f64>>, ResidualError> {
    problem
        .closure_joint_ids()
        .iter()
        .map(|joint_id| {
            let joint = problem
                .mechanism()
                .joints()
                .get(*joint_id)
                .expect("Model contains missing closure joint");
            let i_pose = poses
                .get(joint.i_marker().body_id())
                .expect("closure joint i body has no pose");
            let j_pose = poses
                .get(joint.j_marker().body_id())
                .expect("closure joint j body has no pose");
            let i_twist = *twists.get(&joint.i_marker().body_id()).ok_or(
                ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.i_marker().body_id(),
                },
            )?;
            let j_twist = *twists.get(&joint.j_marker().body_id()).ok_or(
                ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.j_marker().body_id(),
                },
            )?;
            Ok(i_pose.marker_velocity(joint.i_marker(), i_twist)
                - j_pose.marker_velocity(joint.j_marker(), j_twist))
        })
        .collect()
}
pub fn closure_orientation_residual_rates(
    problem: &Model,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<f64>, ResidualError> {
    let mut rates = Vec::new();
    for joint_id in problem.closure_joint_ids() {
        let joint = problem
            .mechanism()
            .joints()
            .get(*joint_id)
            .expect("Model contains missing closure joint");
        let coordinate = problem
            .joint_coordinate(*joint_id)
            .expect("Model contains missing joint coordinate")
            .as_spherical()
            .expect("closure joints are validated to be spherical");
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
            .inverse_transform_vector(&(j_twist.angular_velocity() - i_twist.angular_velocity()));
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
pub fn closure_residual_rates(
    problem: &Model,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<f64>, ResidualError> {
    Ok(closure_position_residual_rates(problem, poses, twists)?
        .into_iter()
        .flat_map(|rate| [rate.x, rate.y, rate.z])
        .chain(closure_orientation_residual_rates(problem, poses, twists)?)
        .collect())
}

#[derive(Debug, Error)]
pub enum ResidualError {
    #[error(transparent)]
    GeneralizedCoordinates(#[from] GeneralizedCoordinatesError),
    #[error("closure joint `{joint_id:?}` body `{body_id:?}` has no twist")]
    MissingBodyTwist { joint_id: JointId, body_id: BodyId },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::yaml::parse_yaml_str;

    fn config(
        problem: &Model,
        candidates: &BTreeMap<JointId, Vector3<f64>>,
    ) -> GeneralizedCoordinates {
        GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), candidates)
    }

    fn poses(problem: &Model, candidates: &BTreeMap<JointId, Vector3<f64>>) -> BodyPoses {
        tree_poses_for_configuration_at(problem, &config(problem, candidates), 0.0).unwrap()
    }

    fn residuals(problem: &Model, candidates: &BTreeMap<JointId, Vector3<f64>>) -> Vec<f64> {
        closure_residuals_at(problem, &poses(problem, candidates), 0.0)
    }

    #[test]
    fn analytic_closure_jacobian_matches_candidate_perturbations() {
        let input = parse_yaml_str(include_str!(
            "../../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let joint_id = JointId::new(1);
        let candidates = BTreeMap::from([(joint_id, Vector3::new(0.2, 0.1, 0.0))]);
        let jacobian =
            closure_jacobian_for_configuration_at(&problem, &config(&problem, &candidates), 0.0)
                .unwrap();
        let step = 1.0e-7;

        for column in 0..jacobian.ncols() {
            let component = problem.free_primary_coordinates()[column].1;
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&joint_id).unwrap()[component] += step;
            backward.get_mut(&joint_id).unwrap()[component] -= step;
            let forward_residuals = residuals(&problem, &forward);
            let backward_residuals = residuals(&problem, &backward);

            for row in 0..jacobian.nrows() {
                let finite_difference =
                    (forward_residuals[row] - backward_residuals[row]) / (2.0 * step);
                assert!((jacobian[(row, column)] - finite_difference).abs() < 1.0e-6);
            }
        }
    }

    #[test]
    fn orientation_closure_jacobian_matches_candidate_perturbations() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let problem = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
        let joint_id = JointId::new(1);
        let candidates = BTreeMap::from([(joint_id, Vector3::new(0.2, 0.1, 0.0))]);
        let jacobian =
            closure_jacobian_for_configuration_at(&problem, &config(&problem, &candidates), 0.0)
                .unwrap();
        let step = 1.0e-7;

        assert_eq!(jacobian.nrows(), 4);
        for column in 0..jacobian.ncols() {
            let component = problem.free_primary_coordinates()[column].1;
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&joint_id).unwrap()[component] += step;
            backward.get_mut(&joint_id).unwrap()[component] -= step;
            let forward_residuals = residuals(&problem, &forward);
            let backward_residuals = residuals(&problem, &backward);

            for row in 0..jacobian.nrows() {
                let finite_difference =
                    (forward_residuals[row] - backward_residuals[row]) / (2.0 * step);
                assert!((jacobian[(row, column)] - finite_difference).abs() < 1.0e-6);
            }
        }
    }

    #[test]
    fn rejects_missing_body_twist_for_closure_rates() {
        let input = parse_yaml_str(include_str!(
            "../../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let poses = poses(&problem, &BTreeMap::new());

        assert!(matches!(
            closure_position_residual_rates(&problem, &poses, &BTreeMap::new()),
            Err(ResidualError::MissingBodyTwist { .. })
        ));
    }

    #[test]
    fn orders_position_rows_before_orientation_rows() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let problem = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
        let poses = poses(&problem, &BTreeMap::new());
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
        let residuals = closure_residuals_at(&problem, &poses, 0.0);
        let rates = closure_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(residuals.len(), 4);
        assert!(residuals.iter().all(|residual| residual.abs() < 1.0e-12));
        assert_eq!(rates.len(), 4);
        assert!(rates[..3].iter().all(|rate| rate.abs() < 1.0e-12));
        assert!((rates[3] + 1.0).abs() < 1.0e-12);
    }
}
