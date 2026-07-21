use std::f64::consts::FRAC_PI_2;

use nalgebra::{UnitQuaternion, Vector3};

use mbs_solver::io::yaml::{YamlModel, read_yaml_str};
use mbs_solver::kinematics::forward::update_body_poses;
use mbs_solver::kinematics::state::{JointCoordinates, KinematicState};
use mbs_solver::model::{Bodies, BodyId, JointId, Joints};

const TOLERANCE: f64 = 1.0e-10;

#[test]
fn simple_forward_reproduce_reference_pose() {
    // Create Basic Setup
    let (_, joints, mut state) = setup(include_str!(
        "../examples/test_case_simple_spherical_forward.yaml"
    ));

    // Extract the expected pose for body 1 from the reference state.
    let expected_pose = state
        .body_poses
        .get(&BodyId(1))
        .expect("reference pose must exist")
        .clone();

    // Check if forward position calculation reproduces the reference pose.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Extract the calculated pose for body 1 after forward position calculation.
    let calculated_pose = state
        .body_poses
        .get(&BodyId(1))
        .expect("calculated pose must exist");

    // Calculate the position error between the expected and calculated poses.
    let position_error = (calculated_pose.position - expected_pose.position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    // Check i and j markers of the spherical joint are coincide
    // after forward position calculation.
    assert_joint_closed(&joints, &state);
}

#[test]
fn simple_forward_changed_coordinates_create_new_pose() {
    // Create Basic Setup
    let (_, joints, mut state) = setup(include_str!(
        "../examples/test_case_simple_spherical_forward.yaml"
    ));

    // Extract joint J1
    let joint = joints
        .primary()
        .next()
        .expect("one primary joint must exist");
    let key = joint.key();

    // Extract coordinates as mutable reference
    let coordinates = match state
        .primary_coordinates
        .get_mut(&key)
        .expect("J1 coordinates must exist")
    {
        JointCoordinates::Spherical(coordinates) => coordinates,
    };

    // Set new joint coordinates
    // -> Rotation arround the y-axis by 90 degrees (pi/2 radians).
    coordinates.relative_orientation =
        UnitQuaternion::from_axis_angle(&Vector3::<f64>::y_axis(), FRAC_PI_2);

    // Update pose based on the changed joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Extracting new pose for body 1
    let calculated_pose = state.body_poses.get(&BodyId(1)).expect("body 1 must exist");
    let expected_position = Vector3::new(-100.0, 0.0, 0.0);
    let position_error = (calculated_pose.position - expected_position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    // Check i and j markers of the spherical joint are coincide
    // after forward position calculation.
    assert_joint_closed(&joints, &state);
}

#[test]
fn two_body_forward_changed_coordinates_create_new_pose() {
    // Create Basic Setup with two bodies and two spherical joints
    let (bodies, joints, mut state) = setup(include_str!(
        "../examples/test_case_two_body_spherical_forward.yaml"
    ));

    // Check if the setup is correct
    assert_eq!(joints.primary().count(), 2, "two primary joints must exist");
    assert_eq!(bodies.iter().skip(1).count(), 2, "two bodies must exist");

    // Define new joint coordinates for both joints
    let orientation_joint_1 = UnitQuaternion::from_axis_angle(&Vector3::<f64>::y_axis(), FRAC_PI_2);
    let orientation_joint_2 = UnitQuaternion::from_axis_angle(&Vector3::<f64>::x_axis(), FRAC_PI_2);

    // Extract joints in loop
    for joint in joints.primary() {
        let key = joint.key();

        // Extract coordinates as mutable reference
        let coordinates = match state
            .primary_coordinates
            .get_mut(&key)
            .expect("Coordinates must exist")
        {
            JointCoordinates::Spherical(coordinates) => coordinates,
        };

        let new_orientation = match joint.id() {
            JointId(1) => orientation_joint_1,
            JointId(2) => orientation_joint_2,
            _ => panic!("Unexpected joint id"),
        };

        // Set new joint coordinates
        // -> Rotation arround the local y-axis by 90 degrees (pi/2 radians)
        // -> Rotation arround the local x-axis by 90 degrees (pi/2 radians)
        coordinates.relative_orientation = new_orientation;
    }

    // Update new body poses based on the changed joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Iterate over bodies to check if the new poses are as expected
    for body in bodies.iter() {
        let body_pose = state
            .body_poses
            .get(&body.id())
            .expect("body pose must exist");

        let expected_position = match body.id() {
            BodyId(0) => Vector3::new(0.0, 0.0, 0.0),
            BodyId(1) => Vector3::new(-100.0, 0.0, 0.0),
            BodyId(2) => Vector3::new(-200.0, 100.0, 0.0),
            _ => panic!("Unexpected body id"),
        };
        let position_error = (body_pose.position - expected_position).norm();

        assert!(
            position_error < TOLERANCE,
            "position error was {position_error}"
        );

        let expected_orientation = match body.id() {
            BodyId(0) => UnitQuaternion::identity(),
            BodyId(1) => orientation_joint_1,
            BodyId(2) => orientation_joint_1 * orientation_joint_2,
            _ => panic!("unexpected body id"),
        };

        let orientation_error = body_pose.orientation.angle_to(&expected_orientation);

        assert!(
            orientation_error < TOLERANCE,
            "orientation error for body {:?} was {orientation_error}",
            body.id()
        );

        // Check i and j markers of the spherical joint are coincide
        // after forward position calculation.
        assert_joint_closed(&joints, &state);
    }
}

// Setup Function
pub fn setup(yaml: &str) -> (Bodies, Joints, KinematicState) {
    let input: YamlModel = read_yaml_str(yaml).expect("YAML must parse");

    let (hardpoints, body_specs, joint_specs) = input
        .into_model_parts()
        .expect("model conversion must succeed");

    let bodies = Bodies::build(&body_specs, &hardpoints).expect("body construction must succeed");
    let joints = Joints::build(&joint_specs, &bodies).expect("joint construction must succeed");
    let state =
        KinematicState::from_reference(&bodies, &joints).expect("reference state must build");

    (bodies, joints, state)
}

fn assert_joint_closed(joints: &Joints, state: &KinematicState) {
    // Obtain the only primary joint.
    for joint in joints.primary() {
        let pose_i = state
            .body_poses
            .get(&joint.i_endpoint().body_id)
            .expect("endpoint i body must have a pose");

        let pose_j = state
            .body_poses
            .get(&joint.j_endpoint().body_id)
            .expect("endpoint j body must have a pose");

        // Transform both body-local markers into global coordinates.
        let point_i = joint.i_endpoint().local_marker.global_position(pose_i);
        let point_j = joint.j_endpoint().local_marker.global_position(pose_j);

        // A spherical both marker positions are coincide.
        let error = (point_j - point_i).norm();

        assert!(
            error < TOLERANCE,
            "spherical joint closure error was {error}"
        );
    }
}
