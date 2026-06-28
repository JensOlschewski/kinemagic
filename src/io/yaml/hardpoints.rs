use std::collections::BTreeMap;

use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlHardpoints {
    pub hardpoints: BTreeMap<String, YamlHardpointCoordinates>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum YamlHardpointCoordinates {
    Named { x: f64, y: f64, z: f64 },
    Array([f64; 3]),
}

impl YamlHardpointCoordinates {
    pub fn to_vector(self) -> Vector3<f64> {
        match self {
            Self::Named { x, y, z } => Vector3::new(x, y, z),
            Self::Array([x, y, z]) => Vector3::new(x, y, z),
        }
    }
}
