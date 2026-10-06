use nalgebra::DMatrix;

use std::collections::BTreeMap;

use crate::model::mechanism::{BodyId, JointId};

pub use crate::data::BodyPoses;

pub use crate::model::spatial::{BodyPose, BodyTwist};

pub type TreeTwistColumns = (Vec<(JointId, usize)>, BTreeMap<BodyId, Vec<BodyTwist>>);

/// Maps global coordinate rates to one body's world-frame twist.
/// Rows 0-2 are linear velocity; rows 3-5 are angular velocity.
pub type BodyJacobian = DMatrix<f64>;

pub type BodyJacobians = BTreeMap<BodyId, BodyJacobian>;

/// Maps global coordinate rates to each body's world-frame twist.
/// Rows 0-2 are linear velocity; rows 3-5 are angular velocity.
pub type TreeBodyJacobians = (Vec<(JointId, usize)>, BodyJacobians);
