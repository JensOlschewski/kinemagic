use std::collections::BTreeMap;

use super::{KinematicsError, coordinates::JointCoordinates, spherical::SphericalCoordinates};
use crate::model::motion::{MotionKind, MotionSpec};
use crate::model::{Bodies, BodyId, BodyPose, JointKey, JointKind, Joints};

pub struct KinematicState {
    primary_coordinates: BTreeMap<JointKey, JointCoordinates>,
    body_poses: BTreeMap<BodyId, BodyPose>,
}

impl KinematicState {
    pub fn get_body_pose(&self, body_id: BodyId) -> Option<&BodyPose> {
        self.body_poses.get(&body_id)
    }

    pub fn body_count(&self) -> usize {
        self.body_poses.len()
    }

    pub fn joint_coordinates(&self, joint_key: JointKey) -> Option<&JointCoordinates> {
        self.primary_coordinates.get(&joint_key)
    }

    pub fn replace_body_poses(&mut self, body_poses: BTreeMap<BodyId, BodyPose>) {
        self.body_poses = body_poses;
    }

    pub fn set_spherical_coordinates(
        &mut self,
        joint_key: JointKey,
        new_coordinates: SphericalCoordinates,
    ) -> Result<(), KinematicsError> {
        let joint_coordinates = self
            .primary_coordinates
            .get_mut(&joint_key)
            .ok_or(KinematicsError::MissingJointCoordinates(joint_key))?;

        match joint_coordinates {
            JointCoordinates::Spherical(current_coordinates) => {
                // Dereference assignment change value
                // behind point not pointer itself!
                *current_coordinates = new_coordinates;

                Ok(())
            }
        }
    }

    pub fn apply_motion(&mut self, motion: &MotionSpec) -> Result<(), KinematicsError> {
        match motion.kind() {
            MotionKind::JointCoordinates { key, coordinates } => {
                let current = self
                    .primary_coordinates
                    .get_mut(key)
                    .ok_or(KinematicsError::MissingJointCoordinates(*key))?;

                match (current, coordinates) {
                    (JointCoordinates::Spherical(current), JointCoordinates::Spherical(new)) => {
                        current.relative_orientation = new.relative_orientation;
                        Ok(())
                    }
                }
            }
        }
    }

    pub fn from_reference(bodies: &Bodies, joints: &Joints) -> Result<Self, KinematicsError> {
        let mut primary_coordinates = BTreeMap::new();

        for joint in joints.primary() {
            let parent = bodies
                .get_by_id(joint.i_endpoint().body_id)
                .ok_or(KinematicsError::MissingBody(joint.i_endpoint().body_id))?;

            let child = bodies
                .get_by_id(joint.j_endpoint().body_id)
                .ok_or(KinematicsError::MissingBody(joint.j_endpoint().body_id))?;

            let marker_i_orientation = joint
                .i_endpoint()
                .local_marker
                .to_global_unit_quaternion(parent.pose());

            let marker_j_orientation = joint
                .j_endpoint()
                .local_marker
                .to_global_unit_quaternion(child.pose());

            let coordinates = match joint.kind() {
                JointKind::Spherical => JointCoordinates::Spherical(SphericalCoordinates {
                    relative_orientation: marker_i_orientation.inverse() * marker_j_orientation,
                }),
                kind => return Err(KinematicsError::UnsupportedJoint(kind)),
            };

            primary_coordinates.insert(joint.key(), coordinates);
        }

        let body_poses = bodies
            .iter()
            .map(|body| (body.id(), body.pose().clone()))
            .collect();

        Ok(Self {
            primary_coordinates,
            body_poses,
        })
    }
}
