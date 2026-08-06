use std::f64::consts::FRAC_PI_2;

use kinemagic::kinematics::spherical::SphericalCoordinates;
use nalgebra::{UnitQuaternion, Vector3};

use kinemagic::io::yaml::{YamlModel, read_yaml_str};
use kinemagic::kinematics::KinematicsError;
use kinemagic::kinematics::coordinates::JointCoordinates;
use kinemagic::kinematics::forward::update_body_poses;
use kinemagic::kinematics::state::KinematicState;
use kinemagic::model::motion::{MotionKind, MotionSpec};
use kinemagic::model::{Bodies, BodyId, JointId, JointKey, JointTopology, Joints, Model};

const TOLERANCE: f64 = 1.0e-10;

#[test]
// This test recreates the reference pose in the yaml
// from a simple one body spherical joint chain.
// Compares the calculated pose with the reference pose in the yaml
// and checks if the joint markers are coincide after forward position calculation
fn one_body_reproduces_reference_pose() {
    let (model, mut state, _) = setup(include_str!("./fixtures/spherical_one_body_reference.yaml"));
    let joints = model.joints();

    let expected_pose = state
        .get_body_pose(BodyId(1))
        .expect("reference pose must exist")
        .clone();

    update_body_poses(&mut state, joints).expect("forward position calculation must succeed");

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

    assert_spherical_joint_positions_coincident(joints, &state);
}

#[test]
fn one_body_reproduces_reference_pose_with_non_aligned_markers() {
    let (model, mut state, _) = setup(include_str!(
        "./fixtures/spherical_one_body_non_aligned_markers.yaml"
    ));
    let joints = model.joints();

    let expected_pose = state
        .get_body_pose(BodyId(1))
        .expect("reference pose must exist")
        .clone();

    // Check if forward position calculation reproduces the reference pose.
    update_body_poses(&mut state, joints).expect("forward position calculation must succeed");

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
    assert_spherical_joint_positions_coincident(joints, &state);
}

#[test]
fn one_body_applies_changed_joint_coordinates() {
    // Create Basic Setup
    let (model, mut state, _) = setup(include_str!("./fixtures/spherical_one_body_reference.yaml"));
    let joints = model.joints();

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
    update_body_poses(&mut state, joints).expect("forward position calculation must succeed");

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
    assert_spherical_joint_positions_coincident(joints, &state);
}

#[test]
fn yaml_simultaneous_motions_update_two_body_chain() {
    let (model, mut state, motions) = setup(include_str!(
        "./fixtures/spherical_two_body_motions_apply.yaml"
    ));

    assert_eq!(motions.len(), 2, "two motions must exist");

    state
        .apply_motions(&motions)
        .expect("motions must apply to state");

    update_body_poses(&mut state, model.joints())
        .expect("forward position calculation must succeed");

    let expected_poses = [
        (BodyId(1), Vector3::new(-100.0, 0.0, 0.0), rotation_y_90()),
        (
            BodyId(2),
            Vector3::new(-200.0, 100.0, 0.0),
            rotation_y_90() * rotation_x_90(),
        ),
    ];

    for (body_id, expected_position, expected_orientation) in expected_poses {
        let pose = state.get_body_pose(body_id).expect("body pose must exist");
        let position_error = (pose.position - expected_position).norm();
        assert!(
            position_error < TOLERANCE,
            "position error for body {body_id:?} was {position_error}"
        );

        let orientation_error = pose.orientation.angle_to(&expected_orientation);
        assert!(
            orientation_error < TOLERANCE,
            "orientation error for body {body_id:?} was {orientation_error}"
        );
    }

    assert_spherical_joint_positions_coincident(model.joints(), &state);
}

#[test]
fn motions_reject_missing_joint() {
    let (_model, mut state, _) =
        setup(include_str!("./fixtures/spherical_one_body_reference.yaml"));

    let key = JointKey {
        topology: JointTopology::Primary,
        id: JointId(99),
    };

    let motion = spherical_motion("Missing joint", key, UnitQuaternion::identity());

    let error = state
        .apply_motions(&[motion])
        .expect_err("missing joint must be rejected");

    assert!(matches!(error, KinematicsError::MissingJointCoordinates(found) if found == key));
}

#[test]
fn motions_reject_duplicate_joint() {
    let (_model, mut state, _) =
        setup(include_str!("./fixtures/spherical_one_body_reference.yaml"));

    let key = JointKey {
        topology: JointTopology::Primary,
        id: JointId(1),
    };

    let motions = [
        spherical_motion("First", key, UnitQuaternion::identity()),
        spherical_motion("Second", key, rotation_y_90()),
    ];

    let error = state
        .apply_motions(&motions)
        .expect_err("duplicate joint motions must be rejected");

    assert!(matches!(error, KinematicsError::DuplicateJointMotion(found) if found == key));
}

fn setup(yaml: &str) -> (Model, KinematicState, Vec<MotionSpec>) {
    let input: YamlModel = read_yaml_str(yaml).expect("YAML must parse");

    let (hardpoints, body_specs, joint_specs, motions) = input
        .into_model_parts()
        .expect("model conversion must succeed");

    let bodies = Bodies::build(&body_specs, &hardpoints).expect("body construction must succeed");
    let joints = Joints::build(&joint_specs, &bodies).expect("joint construction must succeed");
    let model = Model::new(bodies, joints);
    let state = KinematicState::from_reference(model.bodies(), model.joints())
        .expect("reference state must build");

    (model, state, motions)
}

fn spherical_motion(name: &str, key: JointKey, orientation: UnitQuaternion<f64>) -> MotionSpec {
    MotionSpec::new(
        name.to_string(),
        MotionKind::JointCoordinates {
            key,
            coordinates: JointCoordinates::Spherical(SphericalCoordinates {
                relative_orientation: orientation,
            }),
        },
    )
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
