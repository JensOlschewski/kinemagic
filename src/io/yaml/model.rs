use serde::Deserialize;

use super::bodies::YamlBodies;
use super::hardpoints::YamlHardpoints;
use super::joints::YamlJoints;
use crate::model::{BodySpec, Hardpoints, JointSpec};

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlModel {
    pub hardpoints: YamlHardpoints,
    #[serde(default)]
    pub bodies: YamlBodies,
    #[serde(default)]
    pub joints: YamlJoints,
}

impl YamlModel {
    pub fn into_model_parts(self) -> Result<(Hardpoints, Vec<BodySpec>, Vec<JointSpec>), String> {
        let hardpoints = self.hardpoints.into();
        let bodies = self.bodies.try_into()?;
        let joints = self.joints.into();

       Ok((hardpoints, bodies, joints))
    }
}
