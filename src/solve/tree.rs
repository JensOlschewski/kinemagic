use std::collections::BTreeMap;

use nalgebra::{DMatrix, UnitQuaternion, Vector3};

use crate::data::Data;
use crate::data::coordinates::{GeneralizedCoordinates, GeneralizedCoordinatesError};
use crate::model::Model;
use crate::model::coordinates::JointCoordinate;
use crate::model::mechanism::joint::geometry;
use crate::model::mechanism::{BodyId, JointId};
use crate::model::topology::TraversalDirection;
use crate::solve::state::{BodyPose, BodyPoses, BodyTwist, TreeBodyJacobians, TreeTwistColumns};

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
        let child = geometry::child_pose(parent, parent_marker, child_marker, relative_orientation);
        poses.insert(edge.child_body_id(), child);
    }
    data.set_body_poses(BodyPoses { poses });
    Ok(())
}

pub fn tree_twist_columns_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<TreeTwistColumns, GeneralizedCoordinatesError> {
    let (columns, body_jacobians) =
        tree_body_jacobians_for_configuration_at(problem, configuration, time)?;
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

pub fn tree_body_jacobians_for_configuration_at(
    problem: &Model,
    configuration: &GeneralizedCoordinates,
    time: f64,
) -> Result<TreeBodyJacobians, GeneralizedCoordinatesError> {
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
        let (columns, body_jacobians) =
            tree_body_jacobians_for_configuration_at(&problem, &config(&problem, &candidates), 0.0)
                .unwrap();
        let body_jacobian = &body_jacobians[&BodyId::new(1)];
        let step = 1.0e-7;

        for (column, (_, component)) in columns.iter().enumerate() {
            let mut forward = candidates.clone();
            let mut backward = candidates.clone();
            forward.get_mut(&joint_id).unwrap()[*component] += step;
            backward.get_mut(&joint_id).unwrap()[*component] -= step;
            let forward_poses = poses(&problem, &forward);
            let forward_pose = forward_poses.get(BodyId::new(1)).unwrap();
            let backward_poses = poses(&problem, &backward);
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
    fn revolute_joint_rotates_child_body_about_marker_z_axis() {
        let yaml = r#"hardpoints:
  P1: [0.0, 0.0, 0.0]
  P2: [100.0, 0.0, 0.0]
bodies:
  B1:
    body_id: 1
    side: single
    position: [100.0, 0.0, 0.0]
    orientation:
      method: euler
      euler_angles: [0, 0, 0]
    points_on_body: [P2]
joints:
  J1:
    joint_id: 1
    kind: revolute
    i:
      body_id: 0
      position: P1
      orientation:
        method: euler
        euler_angles: [0, 0, 0]
    j:
      body_id: 1
      position: P1
      orientation:
        method: euler
        euler_angles: [0, 0, 0]
motions:
  RotateJ1:
    kind: joint-coordinates
    joint_id: 1
    displacement:
      rot_z: 90.0
"#;
        let problem = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let configuration = GeneralizedCoordinates::new(problem.coordinate_layout());
        let poses = tree_poses_for_configuration_at(&problem, &configuration, 0.0).unwrap();
        let body = poses.get(BodyId::new(1)).unwrap();

        let expected_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);

        assert!(body.orientation().angle_to(&expected_orientation) < 1.0e-12);
        assert!((body.position() - Vector3::new(0.0, 100.0, 0.0)).norm() < 1.0e-9);
    }

    #[test]
    fn preserves_non_contiguous_free_primary_columns() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml")
            .replace("      rot_z: 90", "      rot_x: 10\n      rot_z: 90");
        let problem = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
        let (columns, twists) = tree_twist_columns_for_configuration_at(
            &problem,
            &GeneralizedCoordinates::new(problem.coordinate_layout()),
            0.0,
        )
        .unwrap();

        assert_eq!(columns, vec![(JointId::new(1), 1)]);
        assert!(twists[&BodyId::new(1)][0].angular_velocity().norm() > 0.0);
    }

    #[test]
    fn rejects_configuration_from_different_layout() {
        let yaml = include_str!("../../tests/fixtures/spherical_one_body_closed_loop.yaml");
        let first = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let second = parse_yaml_str(yaml).unwrap().into_model().unwrap();
        let configuration = GeneralizedCoordinates::new(first.coordinate_layout());

        assert!(matches!(
            tree_poses_for_configuration_at(&second, &configuration, 0.0),
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
}
