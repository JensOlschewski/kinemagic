use serde::Deserialize;

use super::bodies::YamlBodies;
use super::hardpoints::YamlHardpoints;
use crate::model::{BodySpec, Hardpoints};

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlModel {
    pub hardpoints: YamlHardpoints,
    #[serde(default)]
    pub bodies: YamlBodies,
}

impl YamlModel {
    pub fn into_model_parts(self) -> (Hardpoints, Vec<BodySpec>) {
        let hardpoints = self.hardpoints.into();
        let bodies = self.bodies.into();

        (hardpoints, bodies)
    }
}
