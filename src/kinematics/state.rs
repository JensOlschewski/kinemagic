use std::collections::BTreeMap;

use super::{KinematicsError, spherical::SphericalCoordinates};
use crate::model::{Bodies, BodyId, BodyPose, JointKey, JointKind, Joints};

pub enum JointCoordinates {
    Spherical(SphericalCoordinates),
}

pub struct KinematicState {
    pub primary_coordinates: BTreeMap<JointKey, JointCoordinates>,
    pub body_poses: BTreeMap<BodyId, BodyPose>,
}

impl KinematicState {
    pub fn from_reference(bodies: &Bodies, joints: &Joints) -> Result<Self, KinematicsError> {
        let mut primary_coordinates = BTreeMap::new();

        for joint in joints.primary() {
            let parent = bodies
                .get_by_id(joint.i_endpoint().body_id)
                .ok_or(KinematicsError::MissingBody(joint.i_endpoint().body_id))?;

            let child = bodies
                .get_by_id(joint.j_endpoint().body_id)
                .ok_or(KinematicsError::MissingBody(joint.j_endpoint().body_id))?;

            let coordinates = match joint.kind() {
                JointKind::Spherical => JointCoordinates::Spherical(SphericalCoordinates {
                    relative_orientation: parent.pose().orientation.inverse()
                        * child.pose().orientation,
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
