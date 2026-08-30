use kinemagic::io::yaml::parse_yaml_str;
use kinemagic::model::{BodyId, JointId};
use kinemagic::problem::{PreparedProblem, prepare};
use kinemagic::solve::{BodyPose, BodyPoses, solve};
use nalgebra::{UnitQuaternion, Vector3};

const TOLERANCE: f64 = 1.0e-12;

#[test]
fn solves_ground_only_problem() {
    let problem = prepared("hardpoints: {}\n");

    let poses = solve(&problem);

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

    let poses = solve(&problem);

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
            .steps()
            .iter()
            .all(|step| step.parent_id() == BodyId::GROUND)
    );

    let poses = solve(&problem);

    assert_reproduces_reference(&problem, &poses);
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

    assert_eq!(problem.steps()[0].joint_id(), JointId::new(1));

    let poses = solve(&problem);

    assert_complete(&problem, &poses);
    assert_joint_constraints(&problem, &poses);

    let first_orientation = problem.steps()[0].relative_orientation();
    let second_orientation = problem.steps()[1].relative_orientation();

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
fn solving_is_repeatable_without_mutating_problem() {
    let problem = prepared(include_str!("fixtures/spherical_two_body_motion.yaml"));

    let first = solve(&problem);
    let second = solve(&problem);

    assert_complete(&problem, &first);
    assert_complete(&problem, &second);

    for body in problem.model().bodies().iter() {
        let a = first.get(body.id()).unwrap();
        let b = second.get(body.id()).unwrap();

        assert_pose_close(a, b.position(), b.orientation());
    }
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
    for step in problem.steps() {
        let joint = problem.model().joints().get(step.joint_id()).unwrap();
        let parent = poses.get(step.parent_id()).unwrap();
        let child = poses.get(step.child_id()).unwrap();

        assert_position_close(
            parent.marker_position(joint.i_marker()),
            child.marker_position(joint.j_marker()),
        );
        assert_orientation_close(
            child.marker_orientation(joint.j_marker()),
            parent.marker_orientation(joint.i_marker()) * step.relative_orientation(),
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
