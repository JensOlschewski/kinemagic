//! Changing values: generalized coordinates (q) and computed results.

pub mod coordinates;

use std::collections::BTreeMap;

use crate::model::mechanism::BodyId;
use crate::model::spatial::BodyPose;

/// Poses of every body, including Ground, for one solved configuration.
pub struct BodyPoses {
    pub(crate) poses: BTreeMap<BodyId, BodyPose>,
}

impl BodyPoses {
    pub fn get(&self, body_id: BodyId) -> Option<&BodyPose> {
        self.poses.get(&body_id)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&BodyId, &BodyPose)> {
        self.poses.iter()
    }
}

/// Computed results from the last successful evaluation.
#[derive(Default)]
pub struct Data {
    body_poses: Option<BodyPoses>,
}

impl Data {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn body_poses(&self) -> Option<&BodyPoses> {
        self.body_poses.as_ref()
    }

    pub(crate) fn set_body_poses(&mut self, poses: BodyPoses) {
        self.body_poses = Some(poses);
    }

    pub(crate) fn into_body_poses(self) -> Option<BodyPoses> {
        self.body_poses
    }
}
