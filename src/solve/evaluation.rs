use std::collections::BTreeMap;

use nalgebra::{DMatrix, Matrix3, UnitQuaternion, Vector3};
use thiserror::Error;

use crate::data::Data;
use crate::data::coordinates::{GeneralizedCoordinates, GeneralizedCoordinatesError};
use crate::model::Model;
use crate::model::mechanism::joint::spherical;
use crate::model::mechanism::{BodyId, JointId, JointKind};
use crate::model::topology::TraversalDirection;
use crate::solve::state::{BodyPose, BodyPoses, BodyTwist, TreeBodyJacobians, TreeTwistColumns};

pub fn tree_poses(problem: &Model) -> BodyPoses {
    tree_poses_at(problem, 0.0)
}

pub fn tree_poses_at(problem: &Model, time: f64) -> BodyPoses {
    tree_poses_for_configuration_at(
        problem,
        &GeneralizedCoordinates::new(problem.coordinate_layout()),
        time,
    )
    .expect("new configuration matches problem layout")
}

pub fn tree_poses_for_candidates(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> BodyPoses {
    tree_poses_for_candidates_at(problem, candidates, 0.0)
}

pub fn tree_poses_for_candidates_at(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> BodyPoses {
    tree_poses_for_configuration_at(
        problem,
        &GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), candidates),
        time,
    )
    .expect("candidate configuration matches problem layout")
}

pub fn tree_poses_for_configuration(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
) -> Result<BodyPoses, GeneralizedCoordinatesError> {
    tree_poses_for_configuration_at(problem, configuration, 0.0)
}

pub fn tree_poses_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<BodyPoses, GeneralizedCoordinatesError> {
    let mut data = Data::new();
    evaluate_tree_poses_at(problem, configuration, time, &mut data)?;
    Ok(data
        .into_body_poses()
        .expect("evaluation produced body poses"))
}

/// Updates body poses in `data` for the given configuration and time.
/// A layout error leaves previous results unchanged.
pub fn evaluate_tree_poses_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
    data: &mut Data,
) -> Result<(), GeneralizedCoordinatesError> {
    configuration.validate_layout(problem.coordinate_layout())?;
    let mut poses = BTreeMap::new();
    poses.insert(
        BodyId::GROUND,
        BodyPose::new(Vector3::zeros(), UnitQuaternion::identity()),
    );
    for edge in problem.tree_edges() {
        let joint = problem
            .mechanism()
            .joints()
            .get(edge.joint_id())
            .expect("Model contains missing joint");
        let parent = poses
            .get(&edge.parent_body_id())
            .expect("Model contains unsolved parent");
        let (parent_marker, child_marker) = match edge.direction() {
            TraversalDirection::IToJ => (joint.i_marker(), joint.j_marker()),
            TraversalDirection::JToI => (joint.j_marker(), joint.i_marker()),
        };
        let coordinate = problem
            .joint_coordinate(edge.joint_id())
            .expect("Model contains missing joint coordinate");
        let orientation =
            coordinate.relative_orientation_at(time, configuration.joint_values(edge.joint_id()));
        let relative_orientation = match edge.direction() {
            TraversalDirection::IToJ => orientation,
            TraversalDirection::JToI => orientation.inverse(),
        };
        let child = match joint.kind() {
            JointKind::Spherical => spherical::spherical_child_pose(
                parent,
                parent_marker,
                child_marker,
                relative_orientation,
            ),
        };
        poses.insert(edge.child_body_id(), child);
    }
    data.set_body_poses(BodyPoses { poses });
    Ok(())
}

pub fn tree_twist_columns(problem: &Model) -> TreeTwistColumns {
    tree_twist_columns_for_configuration(
        problem,
        &GeneralizedCoordinates::new(problem.coordinate_layout()),
    )
    .expect("new configuration matches problem layout")
}
pub fn tree_twist_columns_for_candidates(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> TreeTwistColumns {
    tree_twist_columns_for_candidates_at(problem, candidates, 0.0)
}
pub fn tree_twist_columns_for_candidates_at(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> TreeTwistColumns {
    tree_twist_columns_for_configuration_at(
        problem,
        &GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), candidates),
        time,
    )
    .expect("candidate configuration matches problem layout")
}
pub fn tree_twist_columns_for_configuration(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
) -> Result<TreeTwistColumns, GeneralizedCoordinatesError> {
    tree_twist_columns_for_configuration_at(problem, configuration, 0.0)
}
pub fn tree_twist_columns_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<TreeTwistColumns, GeneralizedCoordinatesError> {
    configuration.validate_layout(problem.coordinate_layout())?;
    tree_twist_columns_for_columns_at(
        problem,
        configuration,
        problem.free_primary_coordinates(),
        false,
        time,
    )
}
fn tree_twist_columns_for_columns_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    columns: Vec<(JointId, usize)>,
    include_prescribed: bool,
    time: f64,
) -> Result<TreeTwistColumns, GeneralizedCoordinatesError> {
    let (columns, body_jacobians) = tree_body_jacobians_for_columns_at(
        problem,
        configuration,
        columns,
        include_prescribed,
        time,
    )?;
    let twists = body_jacobians
        .into_iter()
        .map(|(body_id, jacobian)| {
            let twists = (0..jacobian.ncols())
                .map(|column| {
                    BodyTwist::new(
                        jacobian.fixed_view::<3, 1>(0, column).into_owned(),
                        jacobian.fixed_view::<3, 1>(3, column).into_owned(),
                    )
                })
                .collect();
            (body_id, twists)
        })
        .collect();
    Ok((columns, twists))
}

pub fn tree_body_jacobians(problem: &Model) -> TreeBodyJacobians {
    tree_body_jacobians_for_configuration(
        problem,
        &GeneralizedCoordinates::new(problem.coordinate_layout()),
    )
    .expect("new configuration matches problem layout")
}
pub fn tree_body_jacobians_for_candidates(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> TreeBodyJacobians {
    tree_body_jacobians_for_candidates_at(problem, candidates, 0.0)
}
pub fn tree_body_jacobians_for_candidates_at(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> TreeBodyJacobians {
    tree_body_jacobians_for_configuration_at(
        problem,
        &GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), candidates),
        time,
    )
    .expect("candidate configuration matches problem layout")
}
pub fn tree_body_jacobians_for_configuration(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
) -> Result<TreeBodyJacobians, GeneralizedCoordinatesError> {
    tree_body_jacobians_for_configuration_at(problem, configuration, 0.0)
}
pub fn tree_body_jacobians_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<TreeBodyJacobians, GeneralizedCoordinatesError> {
    configuration.validate_layout(problem.coordinate_layout())?;
    tree_body_jacobians_for_columns_at(
        problem,
        configuration,
        problem.free_primary_coordinates(),
        false,
        time,
    )
}
pub(crate) fn tree_body_jacobians_for_columns_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    columns: Vec<(JointId, usize)>,
    include_prescribed: bool,
    time: f64,
) -> Result<TreeBodyJacobians, GeneralizedCoordinatesError> {
    let poses = tree_poses_for_configuration_at(problem, configuration, time)?;
    let mut body_jacobians = BTreeMap::from([(BodyId::GROUND, DMatrix::zeros(6, columns.len()))]);
    for edge in problem.tree_edges() {
        let joint = problem
            .mechanism()
            .joints()
            .get(edge.joint_id())
            .expect("Model contains missing joint");
        let parent_pose = poses
            .get(edge.parent_body_id())
            .expect("Model contains unsolved parent");
        let child_pose = poses
            .get(edge.child_body_id())
            .expect("Model contains unsolved child");
        let parent_jacobian = body_jacobians
            .get(&edge.parent_body_id())
            .expect("Model contains missing parent Jacobian");
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
        let coordinate = problem
            .joint_coordinate(edge.joint_id())
            .expect("Model contains missing joint coordinate");
        let displacement =
            coordinate.resolve_displacement_at(time, configuration.joint_values(edge.joint_id()));
        let mut relative_angular_velocity = DMatrix::zeros(3, columns.len());
        for (column, (joint_id, component)) in columns.iter().enumerate() {
            let mut displacement_rate = Vector3::zeros();
            if *joint_id == edge.joint_id()
                && (include_prescribed || coordinate.free_component_indices().contains(component))
            {
                displacement_rate[*component] = 1.0;
            }
            let angular_velocity = match edge.direction() {
                TraversalDirection::IToJ => {
                    coordinate.relative_angular_velocity(displacement, displacement_rate)
                }
                TraversalDirection::JToI => {
                    coordinate.reverse_relative_angular_velocity(displacement, displacement_rate)
                }
            };
            relative_angular_velocity.set_column(column, &angular_velocity);
        }
        let parent_linear_velocity = parent_jacobian.fixed_rows::<3>(0).into_owned();
        let parent_angular_velocity = parent_jacobian.fixed_rows::<3>(3).into_owned();
        let child_angular_velocity = &parent_angular_velocity
            + parent_marker_orientation.to_rotation_matrix().matrix() * relative_angular_velocity;
        let child_linear_velocity = parent_linear_velocity
            - parent_marker_offset.cross_matrix() * parent_angular_velocity
            + child_marker_offset.cross_matrix() * &child_angular_velocity;
        let mut child_jacobian = DMatrix::zeros(6, columns.len());
        child_jacobian
            .rows_mut(0, 3)
            .copy_from(&child_linear_velocity);
        child_jacobian
            .rows_mut(3, 3)
            .copy_from(&child_angular_velocity);
        body_jacobians.insert(edge.child_body_id(), child_jacobian);
    }
    Ok((columns, body_jacobians))
}

pub fn closure_jacobian(problem: &Model) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_configuration(
        problem,
        &GeneralizedCoordinates::new(problem.coordinate_layout()),
    )
}
pub fn closure_jacobian_for_candidates(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_candidates_at(problem, candidates, 0.0)
}
pub fn closure_jacobian_for_candidates_at(
    problem: &Model,
    candidates: &BTreeMap<JointId, Vector3<f64>>,
    time: f64,
) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_configuration_at(
        problem,
        &GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), candidates),
        time,
    )
}
pub fn closure_jacobian_for_configuration(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
) -> Result<DMatrix<f64>, ResidualError> {
    closure_jacobian_for_configuration_at(problem, configuration, 0.0)
}
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
            .expect("Model contains missing joint coordinate");
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
        let i_marker_offset = i_pose
            .orientation()
            .transform_vector(&joint.i_marker().position());
        let j_marker_offset = j_pose
            .orientation()
            .transform_vector(&joint.j_marker().position());
        let position_block = i_jacobian.fixed_rows::<3>(0).into_owned()
            - i_marker_offset.cross_matrix() * i_jacobian.fixed_rows::<3>(3)
            - j_jacobian.fixed_rows::<3>(0).into_owned()
            + j_marker_offset.cross_matrix() * j_jacobian.fixed_rows::<3>(3);
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
pub fn closure_orientation_residuals(problem: &Model, poses: &BodyPoses) -> Vec<f64> {
    closure_orientation_residuals_at(problem, poses, 0.0)
}
fn closure_orientation_residuals_at(problem: &Model, poses: &BodyPoses, time: f64) -> Vec<f64> {
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
                .expect("Model contains missing joint coordinate");
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
pub(crate) fn closure_residuals_at(problem: &Model, poses: &BodyPoses, time: f64) -> Vec<f64> {
    closure_position_residuals(problem, poses)
        .into_iter()
        .flat_map(|residual| [residual.x, residual.y, residual.z])
        .chain(closure_orientation_residuals_at(problem, poses, time))
        .collect()
}
pub fn closure_residuals(problem: &Model, poses: &BodyPoses) -> Vec<f64> {
    closure_residuals_at(problem, poses, 0.0)
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
            .expect("Model contains missing joint coordinate");
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

    #[test]
    fn analytic_closure_jacobian_matches_candidate_perturbations() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let joint_id = JointId::new(1);
        let candidates = BTreeMap::from([(joint_id, Vector3::new(0.2, 0.1, 0.0))]);
        let jacobian = closure_jacobian_for_candidates(&problem, &candidates).unwrap();
        let step = 1.0e-7;

        for column in 0..jacobian.ncols() {
            let component = problem.free_primary_coordinates()[column].1;
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&joint_id).unwrap()[component] += step;
            backward.get_mut(&joint_id).unwrap()[component] -= step;
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
    fn orientation_closure_jacobian_matches_candidate_perturbations() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let problem = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
        let joint_id = JointId::new(1);
        let candidates = BTreeMap::from([(joint_id, Vector3::new(0.2, 0.1, 0.0))]);
        let jacobian = closure_jacobian_for_candidates(&problem, &candidates).unwrap();
        let step = 1.0e-7;

        assert_eq!(jacobian.nrows(), 4);
        for column in 0..jacobian.ncols() {
            let component = problem.free_primary_coordinates()[column].1;
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&joint_id).unwrap()[component] += step;
            backward.get_mut(&joint_id).unwrap()[component] -= step;
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
    fn body_jacobian_matches_marker_offset_pose_perturbations() {
        let input = parse_yaml_str(
            r#"hardpoints:
  P1: [1.0, -2.0, 3.0]
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
        euler_angles: [0, 0, 0]
    j:
      body_id: 0
      position: P1
      orientation:
        method: euler
        euler_angles: [0, 0, 0]
"#,
        )
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let joint_id = JointId::new(1);
        let candidates = BTreeMap::from([(joint_id, Vector3::zeros())]);
        let (columns, body_jacobians) = tree_body_jacobians_for_candidates(&problem, &candidates);
        let body_jacobian = &body_jacobians[&BodyId::new(1)];
        let step = 1.0e-7;

        for (column, (_, component)) in columns.iter().enumerate() {
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&joint_id).unwrap()[*component] += step;
            backward.get_mut(&joint_id).unwrap()[*component] -= step;
            let forward_poses = tree_poses_for_candidates(&problem, &forward);
            let forward_pose = forward_poses.get(BodyId::new(1)).unwrap();
            let backward_poses = tree_poses_for_candidates(&problem, &backward);
            let backward_pose = backward_poses.get(BodyId::new(1)).unwrap();
            let linear_finite_difference =
                (forward_pose.position() - backward_pose.position()) / (2.0 * step);
            let angular_finite_difference =
                (forward_pose.orientation() * backward_pose.orientation().inverse()).scaled_axis()
                    / (2.0 * step);

            assert!(
                (body_jacobian.fixed_view::<3, 1>(0, column).into_owned()
                    - linear_finite_difference)
                    .norm()
                    < 1.0e-6
            );
            assert!(
                (body_jacobian.fixed_view::<3, 1>(3, column).into_owned()
                    - angular_finite_difference)
                    .norm()
                    < 1.0e-6
            );
        }
    }

    #[test]
    fn preserves_non_contiguous_free_primary_columns() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
            .replace("      rot_z: 90", "      rot_x: 10\n      rot_z: 90");
        let problem = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
        let (columns, twists) = tree_twist_columns(&problem);

        assert_eq!(columns, vec![(JointId::new(1), 1)]);
        assert!(twists[&BodyId::new(1)][0].angular_velocity().norm() > 0.0);
    }

    #[test]
    fn legacy_candidate_wrapper_matches_configuration_evaluation() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let candidates = BTreeMap::from([(JointId::new(1), Vector3::new(0.2, 0.1, 0.0))]);
        let configuration =
            GeneralizedCoordinates::from_candidates(problem.coordinate_layout(), &candidates);
        let legacy = tree_poses_for_candidates(&problem, &candidates);
        let dense = tree_poses_for_configuration(&problem, &configuration).unwrap();

        for body in problem.mechanism().bodies().iter() {
            assert!(
                legacy
                    .get(body.id())
                    .unwrap()
                    .orientation()
                    .angle_to(&dense.get(body.id()).unwrap().orientation())
                    < 1.0e-12
            );
            assert!(
                (legacy.get(body.id()).unwrap().position()
                    - dense.get(body.id()).unwrap().position())
                .norm()
                    < 1.0e-12
            );
        }
        assert_eq!(
            closure_jacobian_for_candidates(&problem, &candidates).unwrap(),
            closure_jacobian_for_configuration(&problem, &configuration).unwrap()
        );
    }

    #[test]
    fn rejects_configuration_from_different_layout() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml");
        let first = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let second = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let configuration = GeneralizedCoordinates::new(first.coordinate_layout());

        assert!(matches!(
            tree_poses_for_configuration(&second, &configuration),
            Err(GeneralizedCoordinatesError::LayoutMismatch)
        ));
    }

    #[test]
    fn data_holds_last_successful_pose_evaluation() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml");
        let model = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let other_model = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let mut q = GeneralizedCoordinates::new(model.coordinate_layout());
        let mut data = Data::new();

        assert!(data.body_poses().is_none());
        evaluate_tree_poses_at(&model, &q, 0.0, &mut data).unwrap();
        let original = data
            .body_poses()
            .unwrap()
            .get(BodyId::new(1))
            .unwrap()
            .orientation();

        let column = model
            .coordinate_layout()
            .primary_range(JointId::new(1))
            .unwrap()
            .start;
        q.set_primary(column, 0.2).unwrap();
        evaluate_tree_poses_at(&model, &q, 0.0, &mut data).unwrap();
        let updated = data
            .body_poses()
            .unwrap()
            .get(BodyId::new(1))
            .unwrap()
            .orientation();
        assert!(original.angle_to(&updated) > 0.1);

        assert!(matches!(
            evaluate_tree_poses_at(&other_model, &q, 0.0, &mut data),
            Err(GeneralizedCoordinatesError::LayoutMismatch)
        ));
        let retained = data
            .body_poses()
            .unwrap()
            .get(BodyId::new(1))
            .unwrap()
            .orientation();
        assert!(updated.angle_to(&retained) < 1.0e-12);
    }

    #[test]
    fn rejects_missing_body_twist_for_closure_rates() {
        let input = parse_yaml_str(include_str!(
            "../../tests/fixtures/spherical_one_body_closed_loop.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();
        let problem = input;
        let poses = tree_poses(&problem);

        assert!(matches!(
            closure_position_residual_rates(&problem, &poses, &BTreeMap::new()),
            Err(ResidualError::MissingBodyTwist { .. })
        ));
    }

    #[test]
    fn orders_position_rows_before_orientation_rows() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
        );
        let problem = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
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
        assert!(residuals.iter().all(|residual| residual.abs() < 1.0e-12));
        assert_eq!(rates.len(), 4);
        assert!(rates[..3].iter().all(|rate| rate.abs() < 1.0e-12));
        assert!((rates[3] + 1.0).abs() < 1.0e-12);
    }
}
