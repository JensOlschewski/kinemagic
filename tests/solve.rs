use kinemagic::io::yaml::parse_yaml_str;
use kinemagic::model::{BodyId, JointId};
use kinemagic::problem::{PreparedProblem, TraversalDirection, prepare};
use kinemagic::solve::{
    BodyPose, BodyPoses, SolverProgress, solve, solve_at, solve_at_with_progress,
};
use nalgebra::{UnitQuaternion, Vector3};

const TOLERANCE: f64 = 1.0e-12;

#[test]
fn solves_ground_only_problem() {
    let problem = prepared("hardpoints: {}\n");

    let poses = solve(&problem).unwrap();

    assert_eq!(poses.iter().count(), 1);

    assert_pose_close(
        poses.get(BodyId::GROUND).unwrap(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );
}

#[test]
fn reproduces_one_body_reference_with_non_aligned_markers() {
    let yaml = include_str!("fixtures/spherical_one_body_parse.yaml")
        .replacen("euler_angles: [0, 0, 0]", "euler_angles: [20, 30, 40]", 1)
        .replacen("euler_angles: [0, 0, 0]", "euler_angles: [10, 20, 30]", 1)
        .replacen("euler_angles: [0, 0, 0]", "euler_angles: [-20, 15, 35]", 1);
    let problem = prepared(&yaml);
    let joint = problem.model().joints().get(JointId::new(1)).unwrap();

    assert!(
        joint
            .i_marker()
            .orientation()
            .angle_to(&joint.j_marker().orientation())
            > 1.0e-3
    );

    let poses = solve(&problem).unwrap();

    assert_reproduces_reference(&problem, &poses);
}

#[test]
fn solves_branching_reference_problem() {
    let yaml = include_str!("fixtures/spherical_two_body_parse.yaml").replace(
        "    i:\n      body_id: 1\n      position: P2",
        "    i:\n      body_id: 0\n      position: P2",
    );
    let problem = prepared(&yaml);

    assert!(
        problem
            .tree_edges()
            .iter()
            .all(|step| step.parent_body_id() == BodyId::GROUND)
    );

    let poses = solve(&problem);

    assert_reproduces_reference(&problem, &poses.unwrap());
}

#[test]
fn solves_reordered_chain_with_simultaneous_motions() {
    let yaml = include_str!("fixtures/spherical_two_body_motion.yaml")
        .replace("  J1:", "  ZRoot:")
        .replace("  J2:", "  AChild:");
    let problem = prepared(&yaml);

    assert_eq!(
        problem.model().joints().iter().next().unwrap().id(),
        JointId::new(2)
    );

    assert_eq!(problem.tree_edges()[0].joint_id(), JointId::new(1));

    let poses = solve(&problem).unwrap();

    assert_complete(&problem, &poses);
    assert_joint_constraints(&problem, &poses);

    let first_orientation = problem.tree_edges()[0].relative_orientation();
    let second_orientation = problem.tree_edges()[1].relative_orientation();

    assert_orientation_close(
        poses.get(BodyId::new(1)).unwrap().orientation(),
        first_orientation,
    );

    assert_orientation_close(
        poses.get(BodyId::new(2)).unwrap().orientation(),
        first_orientation * second_orientation,
    );
}

#[test]
fn solves_open_tree_at_positive_and_negative_times() {
    let problem = prepared(include_str!("fixtures/spherical_one_body_time_motion.yaml"));

    let positive = solve_at(&problem, 2.0).unwrap();
    let negative = solve_at(&problem, -2.0).unwrap();

    assert_pose_close(
        positive.get(BodyId::new(1)).unwrap(),
        Vector3::new(-34.202014332567, 0.0, -93.969262078591),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 20.0_f64.to_radians()),
    );
    assert_pose_close(
        negative.get(BodyId::new(1)).unwrap(),
        Vector3::new(34.202014332567, 0.0, -93.969262078591),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -20.0_f64.to_radians()),
    );
}

#[test]
fn solving_is_repeatable_without_mutating_problem() {
    let problem = prepared(include_str!("fixtures/spherical_two_body_motion.yaml"));

    let first = solve(&problem).unwrap();
    let second = solve(&problem).unwrap();

    assert_complete(&problem, &first);
    assert_complete(&problem, &second);

    for body in problem.model().bodies().iter() {
        let a = first.get(body.id()).unwrap();
        let b = second.get(body.id()).unwrap();

        assert_pose_close(a, b.position(), b.orientation());
    }
}

#[test]
fn solves_closed_loop_problem() {
    let yaml = include_str!("../examples/spherical_one_body_closed_loop_motion.yaml");
    let problem = prepared(yaml);

    let poses = solve(&problem).unwrap();

    assert_complete(&problem, &poses);
    assert!(
        kinemagic::solve::closure_position_residuals(&problem, &poses,)
            .iter()
            .all(|residual| residual.norm() < 1.0e-8)
    );
}

#[test]
fn solves_time_dependent_closed_loop_problem() {
    let yaml = include_str!("../examples/spherical_one_body_closed_loop_motion.yaml")
        .replace("      rot_z: 90", "      rot_z: \"5*time\"");
    let problem = prepared(&yaml);
    let poses = solve_at(&problem, 2.0).unwrap();

    assert_complete(&problem, &poses);
    assert!(
        kinemagic::solve::closure_position_residuals(&problem, &poses)
            .iter()
            .all(|residual| residual.norm() < 1.0e-8)
    );
}

#[test]
fn reports_closed_loop_progress() {
    let yaml = format!(
        "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
        include_str!("../examples/spherical_one_body_closed_loop_motion.yaml")
    );
    let problem = prepared(&yaml);
    let mut events = Vec::new();

    solve_at_with_progress(&problem, 0.0, |event| events.push(event)).unwrap();

    assert!(
        events
            .iter()
            .any(|event| matches!(event, SolverProgress::Residual { .. }))
    );
    assert!(
        events
            .iter()
            .any(|event| matches!(event, SolverProgress::Converged { .. }))
    );
}

#[test]
fn solves_closed_loop_with_prescribed_orientation_constraint() {
    let yaml = format!(
        "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
        include_str!("../examples/spherical_one_body_closed_loop_motion.yaml")
    );
    let problem = prepared(&yaml);
    let poses = solve(&problem).unwrap();

    assert!(
        kinemagic::solve::closure_orientation_residuals(&problem, &poses)
            .iter()
            .all(|residual| residual.abs() < 1.0e-8)
    );
}

#[test]
fn solves_closed_loop_with_equivalent_large_angle_constraint() {
    let yaml = format!(
        "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: 270\n",
        include_str!("../examples/spherical_one_body_closed_loop_motion.yaml")
    );
    let problem = prepared(&yaml);
    let poses = solve(&problem).unwrap();

    assert!(
        kinemagic::solve::closure_orientation_residuals(&problem, &poses)
            .iter()
            .all(|residual| residual.abs() < 1.0e-8)
    );
}

#[test]
fn rejects_over_prescribed_closed_loop_problem() {
    let yaml = include_str!("../examples/spherical_one_body_closed_loop_motion.yaml").replace(
        "      rot_z: 90",
        "      rot_x: 10\n      rot_y: 20\n      rot_z: 90",
    );
    let problem = prepared(&yaml);

    assert!(matches!(
        solve(&problem),
        Err(kinemagic::solve::SolverError::Jacobian(
            kinemagic::solve::JacobianError::InsufficientCandidateRank { .. }
        ))
    ));
}

fn prepared(yaml: &str) -> PreparedProblem {
    prepare(parse_yaml_str(yaml).unwrap().into_input().unwrap()).unwrap()
}

fn assert_reproduces_reference(problem: &PreparedProblem, poses: &BodyPoses) {
    assert_complete(problem, poses);

    for body in problem.model().bodies().iter() {
        assert_pose_close(
            poses.get(body.id()).unwrap(),
            body.position(),
            body.orientation(),
        );
    }
}

fn assert_complete(problem: &PreparedProblem, poses: &BodyPoses) {
    assert_eq!(
        poses.iter().count(),
        problem.model().bodies().iter().count()
    );
    assert!(
        problem
            .model()
            .bodies()
            .iter()
            .all(|body| poses.get(body.id()).is_some())
    );
}

fn assert_joint_constraints(problem: &PreparedProblem, poses: &BodyPoses) {
    for edge in problem.tree_edges() {
        let joint = problem.model().joints().get(edge.joint_id()).unwrap();
        let parent = poses.get(edge.parent_body_id()).unwrap();
        let child = poses.get(edge.child_body_id()).unwrap();

        let (parent_marker, child_marker, relative_orientation) = match edge.direction() {
            TraversalDirection::IToJ => (
                joint.i_marker(),
                joint.j_marker(),
                edge.relative_orientation(),
            ),

            TraversalDirection::JToI => (
                joint.j_marker(),
                joint.i_marker(),
                edge.relative_orientation().inverse(),
            ),
        };

        assert_position_close(
            parent.marker_position(parent_marker),
            child.marker_position(child_marker),
        );

        assert_orientation_close(
            child.marker_orientation(child_marker),
            parent.marker_orientation(parent_marker) * relative_orientation,
        );
    }
}

fn assert_pose_close(
    actual: &BodyPose,
    expected_position: Vector3<f64>,
    expected_orientation: UnitQuaternion<f64>,
) {
    assert_position_close(actual.position(), expected_position);
    assert_orientation_close(actual.orientation(), expected_orientation);
}

fn assert_position_close(actual: Vector3<f64>, expected: Vector3<f64>) {
    assert!((actual - expected).norm() < TOLERANCE);
}

fn assert_orientation_close(actual: UnitQuaternion<f64>, expected: UnitQuaternion<f64>) {
    assert!(actual.angle_to(&expected) < TOLERANCE);
}
