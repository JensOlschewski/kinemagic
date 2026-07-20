use std::collections::BTreeMap;

use nalgebra::{Quaternion, UnitQuaternion};
use serde::Deserialize;

use crate::io::yaml::hardpoints::YamlHardpointCoordinates;
use crate::io::yaml::side::YamlSide;
use crate::model::{BodyId, BodySpec, BodyPose};

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
                let orientation = match body.orientation {
                    Some(yaml_orientation) => yaml_orientation.to_unit_quaternion()?,
                    None => UnitQuaternion::identity(),
                };

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
    pub fn to_unit_quaternion(self) -> Result<UnitQuaternion<f64>, String> {
        let [e0, e1, e2, e3] = match self {
            Self::EulerParameters { euler_parameters } => euler_parameters,
            Self::Array(values) => values,
        };

        UnitQuaternion::try_new(Quaternion::new(e0, e1, e2, e3), 1e-6)
            .ok_or_else(|| "Body quaternion is zero or contains non-finite values".to_string())
    }
}
