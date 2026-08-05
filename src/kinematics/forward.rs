use super::KinematicsError;
use crate::kinematics::coordinates::JointCoordinates;
use crate::kinematics::spherical;
use crate::kinematics::state::KinematicState;
use crate::model::joints::{Joint, JointKind};
use crate::model::{BodyId, BodyPose, Joints};
use std::collections::BTreeMap;

pub fn update_body_poses(
    state: &mut KinematicState,
    joints: &Joints,
) -> Result<(), KinematicsError> {
    let expected_body_count = state.body_count();

    let ground_pose = state
        .get_body_pose(BodyId::GROUND)
        .cloned()
        .ok_or(KinematicsError::MissingBody(BodyId::GROUND))?;

    let mut new_poses = BTreeMap::new();
    new_poses.insert(BodyId::GROUND, ground_pose);

    for joint in joints.primary() {
        let parent_id = joint.i_endpoint().body_id;
        let child_id = joint.j_endpoint().body_id;

        // The ID-order requires the parent to be resolved already.
        let parent_pose = new_poses
            .get(&parent_id)
            .ok_or(KinematicsError::InvalidPrimaryTree)?;

        // A child must have exactly one primary parent.
        if new_poses.contains_key(&child_id) {
            return Err(KinematicsError::InvalidPrimaryTree);
        }

        let coordinates = state
            .joint_coordinates(joint.key())
            .ok_or(KinematicsError::MissingJointCoordinates(joint.key()))?;

        let calculated_pose = calculate_child_pose(joint, parent_pose, coordinates)?;

        new_poses.insert(child_id, calculated_pose);
    }

    // Detect bodies that are not connected through primary joints.
    if new_poses.len() != expected_body_count {
        return Err(KinematicsError::InvalidPrimaryTree);
    }

    // Commit only after the complete calculation succeeds.
    state.replace_body_poses(new_poses);

    Ok(())
}

pub fn calculate_child_pose(
    joint: &Joint,
    parent_pose: &BodyPose,
    coordinates: &JointCoordinates,
) -> Result<BodyPose, KinematicsError> {
    match (joint.kind(), coordinates) {
        (JointKind::Spherical, JointCoordinates::Spherical(coordinates)) => {
            Ok(spherical::explicit_position(
                parent_pose,
                joint.i_endpoint(),
                joint.j_endpoint(),
                coordinates,
            ))
        }
        _ => Err(KinematicsError::UnsupportedJoint(joint.kind())),
    }
}
