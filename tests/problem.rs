use kinemagic::io::yaml::{YamlError, parse_yaml_str};
use kinemagic::model::mechanism::{BodyId, JointId};
use kinemagic::model::{ModelError, coordinates::JointCoordinateError};
use kinemagic::solve::{tree_body_jacobians_for_candidates, tree_poses_for_candidates};
use nalgebra::{UnitQuaternion, Vector3};

#[test]
fn prepares_ordered_edges_with_resolved_coordinates() {
    let yaml = include_str!("fixtures/spherical_two_body_motion.yaml");
    let input = parse_yaml_str(yaml).unwrap().into_model().unwrap();

    let problem = input;

    assert_eq!(problem.mechanism().bodies().iter().count(), 3);
    assert_eq!(problem.tree_edges().len(), 2);
    assert_eq!(problem.tree_edges()[0].parent_body_id(), BodyId::GROUND);
    assert_eq!(problem.tree_edges()[0].child_body_id(), BodyId::new(1));
    assert_eq!(problem.tree_edges()[0].joint_id(), JointId::new(1));
    assert_eq!(problem.tree_edges()[1].parent_body_id(), BodyId::new(1));
    assert_eq!(problem.tree_edges()[1].child_body_id(), BodyId::new(2));
    assert_eq!(problem.tree_edges()[1].joint_id(), JointId::new(2));

    let expected_joint_1 =
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    let expected_joint_2 =
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);

    assert!(
        problem
            .joint_coordinate(problem.tree_edges()[0].joint_id())
            .unwrap()
            .relative_orientation()
            .angle_to(&expected_joint_1)
            < 1.0e-12
    );
    assert!(
        problem
            .joint_coordinate(problem.tree_edges()[1].joint_id())
            .unwrap()
            .relative_orientation()
            .angle_to(&expected_joint_2)
            < 1.0e-12
    );
}

#[test]
fn yaml_name_order_does_not_change_prepared_edges() {
    let fixture = include_str!("fixtures/spherical_two_body_parse.yaml");
    let root_first = fixture
        .replace("  J1:", "  ARoot:")
        .replace("joint_id: 1", "joint_id: 9")
        .replace("  J2:", "  ZChild:")
        .replace("joint_id: 2", "joint_id: 1");
    let child_first = fixture
        .replace("  J1:", "  ZRoot:")
        .replace("joint_id: 1", "joint_id: 9")
        .replace("  J2:", "  AChild:")
        .replace("joint_id: 2", "joint_id: 1");
    let root_first = parse_yaml_str(&root_first).unwrap().into_model().unwrap();
    let child_first = parse_yaml_str(&child_first).unwrap().into_model().unwrap();

    let edge_ids = |problem: &kinemagic::model::Model| {
        problem
            .tree_edges()
            .iter()
            .map(|edge| (edge.parent_body_id(), edge.child_body_id(), edge.joint_id()))
            .collect::<Vec<_>>()
    };

    assert_eq!(edge_ids(&root_first), edge_ids(&child_first));
}

#[test]
fn exposes_traversal_direction_for_reversed_edge_rates() {
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
    let edge = &problem.tree_edges()[0];
    let coordinate = problem.joint_coordinate(edge.joint_id()).unwrap();
    let displacement = Vector3::new(0.4, -0.3, 0.2);
    let displacement_rate = Vector3::new(-0.2, 0.5, 0.7);

    assert_eq!(
        edge.direction(),
        kinemagic::model::topology::TraversalDirection::JToI
    );
    assert!(
        coordinate
            .relative_orientation_for(displacement)
            .inverse()
            .angle_to(&UnitQuaternion::from_scaled_axis(displacement).inverse())
            < 1.0e-12
    );
    assert!(
        (coordinate.reverse_relative_angular_velocity(displacement, displacement_rate)
            + coordinate
                .relative_orientation_for(displacement)
                .inverse_transform_vector(
                    &coordinate.relative_angular_velocity(displacement, displacement_rate)
                ))
        .norm()
            < 1.0e-12
    );

    let candidates = std::collections::BTreeMap::from([(edge.joint_id(), displacement)]);
    let poses = tree_poses_for_candidates(&problem, &candidates);
    assert!(
        poses
            .get(BodyId::new(1))
            .unwrap()
            .orientation()
            .angle_to(&UnitQuaternion::from_scaled_axis(displacement).inverse())
            < 1.0e-12
    );

    let (columns, jacobians) = tree_body_jacobians_for_candidates(&problem, &candidates);
    let step = 1.0e-7;
    for (column, (_, component)) in columns.iter().enumerate() {
        let mut forward = candidates.clone();
        let mut backward = candidates.clone();
        forward.get_mut(&edge.joint_id()).unwrap()[*component] += step;
        backward.get_mut(&edge.joint_id()).unwrap()[*component] -= step;
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
        let angular_column = jacobians[&BodyId::new(1)]
            .fixed_view::<3, 1>(3, column)
            .into_owned();
        assert!((angular_column - finite_difference).norm() < 1.0e-6);
    }
}

#[test]
fn exposes_typed_preparation_errors() {
    let yaml = format!(
        "{}\nmotions:\n  Unknown:\n    kind: joint-coordinates\n    joint_id: 99\n    displacement:\n      rot_x: 90.0\n",
        include_str!("fixtures/spherical_one_body_parse.yaml")
    );
    let error = match parse_yaml_str(&yaml).unwrap().into_model() {
        Ok(_) => panic!("unknown motion joint should fail model construction"),
        Err(error) => error,
    };

    assert!(matches!(
        error,
        YamlError::Derive(ModelError::Coordinates(JointCoordinateError::UnknownJoint {
            motion_name,
            joint_id,
        })) if motion_name == "Unknown" && joint_id == JointId::new(99)
    ));
}
