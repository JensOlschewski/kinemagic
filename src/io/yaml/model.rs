//! YAML input format.
//!
//! ```yaml
//! hardpoints:
//!   P1: [0.0, 0.0, 0.0]
//!
//! bodies:
//!   B1:
//!     body_id: 1
//!     side: single
//!     cm: [0.0, 0.0, -100.0]
//!     orientation: [1.0, 0.0, 0.0, 0.0]
//!     points_on_body: [P1]
//!
//! joints:
//!   primary:
//!     J1:
//!       id: 1
//!       type: spherical
//!       i:
//!         body_id: 0
//!         point: P1
//!         orientation:
//!           method: euler
//!           euler_angles: [0.0, 0.0, 0.0]
//!       j:
//!         body_id: 1
//!         point: P1
//!         orientation:
//!           method: euler
//!           euler_angles: [0.0, 0.0, 0.0]
//!
//!   secondary: {}
//!
//! motions:
//!   Motion1:
//!     type: joint-coordinates
//!     joint_topology: primary
//!     joint_id: 1
//!     relative_orientation:
//!       method: quaternion
//!       quaternion: [0.70710678, 0.0, 0.70710678, 0.0]
//! ```
//!
//! Quaternion component order is `[w, x, y, z]`.
//! Euler angles use degrees.

use serde::Deserialize;

use super::bodies::YamlBodies;
use super::hardpoints::YamlHardpoints;
use super::joints::YamlJoints;
use super::motion::YamlMotions;
use crate::model::motion::MotionSpec;
use crate::model::{BodySpec, Hardpoints, JointSpec};

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlModel {
    pub hardpoints: YamlHardpoints,
    #[serde(default)]
    pub bodies: YamlBodies,
    #[serde(default)]
    pub joints: YamlJoints,
    #[serde(default)]
    pub motions: YamlMotions,
}

impl YamlModel {
    pub fn into_model_parts(
        self,
    ) -> Result<(Hardpoints, Vec<BodySpec>, Vec<JointSpec>, Vec<MotionSpec>), String> {
        let hardpoints = self.hardpoints.into();
        let bodies = self.bodies.try_into()?;
        let joints = self.joints.into();
        let motions = self.motions.try_into()?;

        Ok((hardpoints, bodies, joints, motions))
    }
}
