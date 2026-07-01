use std::collections::BTreeMap;

use nalgebra::{Quaternion, UnitQuaternion};
use serde::Deserialize;

use crate::io::yaml::hardpoints::YamlHardpointCoordinates;
use crate::model::{BodyId, BodySpec, Pose, Side};

#[derive(Debug, Clone, PartialEq, Default, Deserialize)]
pub struct YamlBodies(pub BTreeMap<String, YamlBody>);

impl From<YamlBodies> for Vec<BodySpec> {
    fn from(value: YamlBodies) -> Self {
        value
            .0
            .into_iter()
            .map(|(name, body)| BodySpec {
                name,
                id: BodyId(body.body_id),
                side: body.side.into(),
                pose: Pose::new(
                    body.center_of_mass.to_vector(),
                    body.orientation
                        .map(YamlBodyOrientation::to_unit_quaternion)
                        .unwrap_or_else(UnitQuaternion::identity),
                ),
                point_names: body.point_names,
            })
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlBody {
    pub body_id: u32,
    pub side: YamlSide,
    #[serde(rename = "cm")]
    pub center_of_mass: YamlHardpointCoordinates,
    pub orientation: Option<YamlBodyOrientation>,
    #[serde(rename = "points_on_body")]
    pub point_names: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlSide {
    Left,
    Right,
    Single,
}

impl From<YamlSide> for Side {
    fn from(value: YamlSide) -> Self {
        match value {
            YamlSide::Left => Self::Left,
            YamlSide::Right => Self::Right,
            YamlSide::Single => Self::Single,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum YamlBodyOrientation {
    EulerParameters { euler_parameters: [f64; 4] },
    Array([f64; 4]),
}

impl YamlBodyOrientation {
    pub fn to_unit_quaternion(self) -> UnitQuaternion<f64> {
        let [e0, e1, e2, e3] = match self {
            Self::EulerParameters { euler_parameters } => euler_parameters,
            Self::Array(values) => values,
        };

        UnitQuaternion::new_normalize(Quaternion::new(e0, e1, e2, e3))
    }
}
