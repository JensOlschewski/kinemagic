use std::collections::BTreeMap;

use nalgebra::DMatrix;

use crate::model::mechanism::{BodyId, JointId};

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

pub use crate::model::spatial::{BodyPose, BodyTwist};

pub type TreeTwistColumns = (Vec<(JointId, usize)>, BTreeMap<BodyId, Vec<BodyTwist>>);

/// Maps global coordinate rates to one body's world-frame twist.
/// Rows 0-2 are linear velocity; rows 3-5 are angular velocity.
pub type BodyJacobian = DMatrix<f64>;

pub type BodyJacobians = BTreeMap<BodyId, BodyJacobian>;

/// Maps global coordinate rates to each body's world-frame twist.
/// Rows 0-2 are linear velocity; rows 3-5 are angular velocity.
pub type TreeBodyJacobians = (Vec<(JointId, usize)>, BodyJacobians);
