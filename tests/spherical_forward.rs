use std::f64::consts::FRAC_PI_2;

use nalgebra::{UnitQuaternion, Vector3};

use mbs_solver::io::yaml::{YamlModel, read_yaml_str};
use mbs_solver::kinematics::forward::update_body_poses;
use mbs_solver::kinematics::state::{JointCoordinates, KinematicState};
use mbs_solver::model::{Bodies, BodyId, Joints, JointId};

const TOLERANCE: f64 = 1.0e-10;

#[test]
fn simple_forward_reproduce_reference_pose() {
    // Create Basic Setup
    let (bodies, joints, mut state) = setup(include_str!("../examples/simple_spherical_forward.yaml"));

    let expected_pose = state
        .body_poses
        .get(&BodyId(1))
        .expect("reference pose must exist")
        .clone();

    // Recalculate all moving-body poses from ground and joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    let calculated_pose = state
        .body_poses
        .get(&BodyId(1))
        .expect("calculated pose must exist");

    let position_error = (calculated_pose.position - expected_pose.position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    assert_joint_closed(&joints, &state);
}

#[test]
fn simple_forward_changed_coordinates_create_new_pose() {
    // Create Basic Setup
    let (bodies, joints, mut state) = setup(include_str!("../examples/simple_spherical_forward.yaml"));

    // Extract joint
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

    // Set new joint coordinates -> Rotation arround the y-axis by 90 degrees (pi/2 radians).
    coordinates.relative_orientation =
        UnitQuaternion::from_axis_angle(&Vector3::<f64>::y_axis(), FRAC_PI_2);

    // Update new body poses based on the changed joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    // Extracting new pose for body 1
    let body_pose = state.body_poses.get(&BodyId(1)).expect("body 1 must exist");

    let expected_position = Vector3::new(-100.0, 0.0, 0.0);
    let position_error = (body_pose.position - expected_position).norm();

    assert!(
        position_error < TOLERANCE,
        "position error was {position_error}"
    );

    assert_joint_closed(&joints, &state);
}

#[test]
fn two_body_forward_changed_coordinates_create_new_pose() {
    // Create Basic Setup
    let (bodies, joints, mut state) = setup(include_str!("../examples/two_body_spherical_forward.yaml"));

    // Extract joint
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


    let new_orientation = match
        joint.id() {
            JointId(1) => UnitQuaternion::from_axis_angle(&Vector3::<f64>::y_axis(), FRAC_PI_2),
            JointId(2) => UnitQuaternion::from_axis_angle(&Vector3::<f64>::x_axis(), FRAC_PI_2),
            _ => panic!("Unexpected joint id"),
        };

    // Set new joint coordinates -> Rotation arround the y-axis by 90 degrees (pi/2 radians).
    coordinates.relative_orientation = new_orientation;
    }

    // Update new body poses based on the changed joint coordinates.
    update_body_poses(&mut state, &joints).expect("forward position calculation must succeed");

    for body in bodies.iter() {
        let body_pose = state
            .body_poses
            .get(&body.id())
            .expect("body pose must exist");

        let expected_position = match body.id() {
            BodyId(0) => Vector3::new(-100.0, 0.0, 0.0),
            BodyId(1) => Vector3::new(-100.0, 0.0, -100.0),
            _ => panic!("Unexpected body id"),
        };

        let position_error = (body_pose.position - expected_position).norm();

        assert!(
            position_error < TOLERANCE,
            "position error was {position_error}"
        );
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
    let joint = joints
        .primary()
        .next()
        .expect("one primary joint must exist");

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

    // A spherical joint requires both marker positions to coincide.
    let error = (point_j - point_i).norm();

    assert!(
        error < TOLERANCE,
        "spherical joint closure error was {error}"
    );
}
