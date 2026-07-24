use std::f64::consts::FRAC_PI_2;

use mbs_solver::kinematics::spherical::SphericalCoordinates;
use nalgebra::{UnitQuaternion, Vector3};

use mbs_solver::io::yaml::{YamlModel, read_yaml_str};
use mbs_solver::kinematics::forward::update_body_poses;
use mbs_solver::kinematics::state::KinematicState;
use mbs_solver::model::{Bodies, BodyId, JointId, Joints};

const TOLERANCE: f64 = 1.0e-10;

#[test]
// This test recreates the reference pose in the yaml
// from a simple one body spherical joint chain.
// Compares the calculated pose with the reference pose in the yaml
// and checks if the joint markers are coincide after forward position calculation
fn one_body_reproduces_reference_pose() {
    let (_, joints, mut state) =
        setup(include_str!("./fixtures/spherical_one_body_reference.yaml"));

    let expected_pose = state
        .get_body_pose(BodyId(1))
        .expect("reference pose must exist")
        .clone();

    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    let calculated_pose = state
        .get_body_pose(BodyId(1))
        .expect("calculated pose must exist");

    let position_error = (calculated_pose.position - expected_pose.position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    let orientation_error = calculated_pose
        .orientation
        .angle_to(&expected_pose.orientation);

    assert!(
        orientation_error < TOLERANCE,
        "orientation error was {orientation_error}"
    );

    assert_spherical_joint_positions_coincident(&joints, &state);
}

#[test]
fn one_body_reproduces_reference_pose_with_non_aligned_markers() {
    let (_, joints, mut state) = setup(include_str!(
        "./fixtures/spherical_one_body_non_aligned_markers.yaml"
    ));

    let expected_pose = state
        .get_body_pose(BodyId(1))
        .expect("reference pose must exist")
        .clone();

    // Check if forward position calculation reproduces the reference pose.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Extract the calculated pose for body 1 after forward position calculation.
    let calculated_pose = state
        .get_body_pose(BodyId(1))
        .expect("calculated pose must exist");

    // Calculate the position error between the expected and calculated poses.
    let position_error = (calculated_pose.position - expected_pose.position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    let orientation_error = calculated_pose
        .orientation
        .angle_to(&expected_pose.orientation);

    assert!(
        orientation_error < TOLERANCE,
        "orientation error was {orientation_error}"
    );

    // Check i and j markers of the spherical joint are coincide
    // after forward position calculation.
    assert_spherical_joint_positions_coincident(&joints, &state);
}

#[test]
fn one_body_applies_changed_joint_coordinates() {
    // Create Basic Setup
    let (_, joints, mut state) =
        setup(include_str!("./fixtures/spherical_one_body_reference.yaml"));

    // Extract joint J1
    let joint = joints
        .primary()
        .next()
        .expect("one primary joint must exist");

    // Extract coordinates as mutable reference
    state
        .set_spherical_coordinates(
            joint.key(),
            SphericalCoordinates {
                relative_orientation: rotation_y_90(),
            },
        )
        .expect("joint coordinates must exist");

    // Update pose based on the changed joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Extracting new pose for body 1
    let calculated_pose = state.get_body_pose(BodyId(1)).expect("body 1 must exist");
    let expected_position = Vector3::new(-100.0, 0.0, 0.0);
    let position_error = (calculated_pose.position - expected_position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    // Check i and j markers of the spherical joint are coincide
    // after forward position calculation.
    assert_spherical_joint_positions_coincident(&joints, &state);
}

#[test]
fn two_body_chain_applies_changed_joint_coordinates() {
    let (bodies, joints, mut state) =
        setup(include_str!("./fixtures/spherical_two_body_chain.yaml"));

    // Check if the setup is correct
    assert_eq!(joints.primary().count(), 2, "two primary joints must exist");
    assert_eq!(bodies.iter().skip(1).count(), 2, "two bodies must exist");

    // Extract joints in loop
    for joint in joints.primary() {
        let new_orientation = match joint.id() {
            JointId(1) => rotation_y_90(),
            JointId(2) => rotation_x_90(),
            _ => panic!("Unexpected joint id"),
        };

        // Assign quarterions to spherical coordinates
        state
            .set_spherical_coordinates(
                joint.key(),
                SphericalCoordinates {
                    relative_orientation: new_orientation,
                },
            )
            .expect("joint coordinates must exist");
    }

    // Update new body poses based on the changed joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Iterate over bodies to check if the new poses are as expected
    for body in bodies.iter() {
        let body_pose = state
            .get_body_pose(body.id())
            .expect("body pose must exist");

        let expected_position = match body.id() {
            BodyId::GROUND => Vector3::new(0.0, 0.0, 0.0),
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
            BodyId::GROUND => UnitQuaternion::identity(),
            BodyId(1) => rotation_y_90(),
            BodyId(2) => rotation_y_90() * rotation_x_90(),
            _ => panic!("unexpected body id"),
        };

        let orientation_error = body_pose.orientation.angle_to(&expected_orientation);

        assert!(
            orientation_error < TOLERANCE,
            "orientation error for body {:?} was {orientation_error}",
            body.id()
        );
    }

    // Check i and j markers of the spherical joint are coincide
    // after forward position calculation.
    assert_spherical_joint_positions_coincident(&joints, &state);
}

#[test]
fn yaml_motion_joint_coordinates_update_body_pose() {
    // Parse model and motion.
    // Build reference state.
    // Apply motion.joint_coordinates.
    // Run forward position.
    // Assert the changed body position and orientation.
}

fn setup(yaml: &str) -> (Bodies, Joints, KinematicState) {
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

fn assert_spherical_joint_positions_coincident(joints: &Joints, state: &KinematicState) {
    // Obtain the only primary joint.
    for joint in joints.primary() {
        let pose_i = state
            .get_body_pose(joint.i_endpoint().body_id)
            .expect("endpoint i body must have a pose");

        let pose_j = state
            .get_body_pose(joint.j_endpoint().body_id)
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

// -> Rotation arround the local y-axis by 90 degrees (pi/2 radians)
fn rotation_y_90() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2)
}

// -> Rotation arround the local x-axis by 90 degrees (pi/2 radians)
fn rotation_x_90() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2)
}
