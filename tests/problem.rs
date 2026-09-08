use kinemagic::io::yaml::parse_yaml_str;
use kinemagic::model::{BodyId, JointId};
use kinemagic::problem::{JointCoordinateError, PrepareError, prepare};
use nalgebra::{UnitQuaternion, Vector3};

#[test]
fn prepares_ordered_steps_with_resolved_coordinates() {
    let yaml = include_str!("fixtures/spherical_two_body_motion.yaml");
    let input = parse_yaml_str(yaml).unwrap().into_input().unwrap();

    let problem = prepare(input).unwrap();

    assert_eq!(problem.model().bodies().iter().count(), 3);
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
        problem.tree_edges()[0]
            .relative_orientation()
            .angle_to(&expected_joint_1)
            < 1.0e-12
    );
    assert!(
        problem.tree_edges()[1]
            .relative_orientation()
            .angle_to(&expected_joint_2)
            < 1.0e-12
    );
}

#[test]
fn yaml_name_order_does_not_change_prepared_steps() {
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
    let root_first = prepare(parse_yaml_str(&root_first).unwrap().into_input().unwrap()).unwrap();
    let child_first = prepare(parse_yaml_str(&child_first).unwrap().into_input().unwrap()).unwrap();

    let step_ids = |problem: &kinemagic::problem::PreparedProblem| {
        problem
            .tree_edges()
            .iter()
            .map(|step| (step.parent_body_id(), step.child_body_id(), step.joint_id()))
            .collect::<Vec<_>>()
    };

    assert_eq!(step_ids(&root_first), step_ids(&child_first));
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
    .into_input()
    .unwrap();
    let problem = prepare(input).unwrap();
    let edge = &problem.tree_edges()[0];
    let displacement = Vector3::new(0.4, -0.3, 0.2);
    let displacement_rate = Vector3::new(-0.2, 0.5, 0.7);

    assert_eq!(edge.direction(), kinemagic::problem::TraversalDirection::JToI);
    assert!(
        edge.traversal_relative_orientation(displacement)
            .angle_to(&UnitQuaternion::from_scaled_axis(displacement).inverse())
            < 1.0e-12
    );
    assert!(
        (edge.traversal_relative_angular_velocity(displacement, displacement_rate)
            - edge
                .joint_coordinate()
                .reverse_relative_angular_velocity(displacement, displacement_rate))
        .norm()
            < 1.0e-12
    );
}

#[test]
fn exposes_typed_preparation_errors() {
    let yaml = format!(
        "{}\nmotions:\n  Unknown:\n    kind: joint-coordinates\n    joint_id: 99\n    displacement:\n      rot_x: 90.0\n",
        include_str!("fixtures/spherical_one_body_parse.yaml")
    );
    let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();

    let error = match prepare(input) {
        Ok(_) => panic!("unknown motion joint should fail preparation"),
        Err(error) => error,
    };

    assert!(matches!(
        error,
        PrepareError::Coordinates(JointCoordinateError::UnknownJoint {
            motion_name,
            joint_id,
        }) if motion_name == "Unknown" && joint_id == JointId::new(99)
    ));
}
