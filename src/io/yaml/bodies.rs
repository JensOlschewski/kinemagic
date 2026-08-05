use std::collections::BTreeMap;

use nalgebra::{Quaternion, UnitQuaternion};
use serde::Deserialize;

use crate::io::yaml::hardpoints::YamlHardpointCoordinates;
use crate::io::yaml::side::YamlSide;
use crate::model::{BodyId, BodyPose, BodySpec};

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlBody {
    pub body_id: u16,
    pub side: YamlSide,
    #[serde(rename = "cm")]
    pub center_of_mass: YamlHardpointCoordinates,
    pub orientation: Option<YamlBodyOrientation>,
    #[serde(rename = "points_on_body")]
    pub point_names: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Default, Deserialize)]
pub struct YamlBodies(pub BTreeMap<String, YamlBody>);

impl TryFrom<YamlBodies> for Vec<BodySpec> {
    type Error = String;

    fn try_from(value: YamlBodies) -> Result<Self, Self::Error> {
        value
            .0
            .into_iter()
            .map(|(name, body)| {
                let orientation = body
                    .orientation
                    .ok_or_else(|| format!("missing orientation for body '{name}'"))?
                    .into_unit_quaternion()
                    .map_err(|error| format!("invalid orientation for body '{name}': {error}"))?;

                Ok(BodySpec {
                    name,
                    id: BodyId(body.body_id),
                    side: body.side.into(),
                    pose: BodyPose::new(body.center_of_mass.to_vector(), orientation),
                    point_names: body.point_names,
                })
            })
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum YamlBodyOrientation {
    EulerParameters { euler_parameters: [f64; 4] },
    Array([f64; 4]),
}

impl YamlBodyOrientation {
    fn into_unit_quaternion(self) -> Result<UnitQuaternion<f64>, String> {
        let [e0, e1, e2, e3] = match self {
            Self::EulerParameters { euler_parameters } => euler_parameters,
            Self::Array(values) => values,
        };

        let components = [e0, e1, e2, e3];

        if components.iter().any(|value| !value.is_finite()) {
            return Err("body quaternion contains non-finite values".to_string());
        }

        UnitQuaternion::try_new(Quaternion::new(e0, e1, e2, e3), 1e-6)
            .ok_or_else(|| "body quaternion norm is too small".to_string())
    }
}
