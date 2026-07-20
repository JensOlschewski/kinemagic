use ::std::collections::BTreeMap;

use ::serde::Deserialize;

use super::orientation::YamlOrientation;
use crate::model::{BodyId, JointEndpointSpec, JointId, JointKind, JointSpec, JointTopology};

#[derive(Debug, Clone, PartialEq, Default, Deserialize)]

pub struct YamlJoints {
    #[serde(default)]
    pub primary: BTreeMap<String, YamlJoint>,
    #[serde(default)]
    pub secondary: BTreeMap<String, YamlJoint>,
}

impl From<YamlJoints> for Vec<JointSpec> {
    fn from(value: YamlJoints) -> Self {
        let primary = value.primary.into_iter().map(|(name, joint)| JointSpec {
            name,
            id: JointId(joint.id),
            kind: joint.kind.into(),
            topology: JointTopology::Primary,
            i_endpoint: joint.i_endpoint.into(),
            j_endpoint: joint.j_endpoint.into(),
        });

        let secondary = value.secondary.into_iter().map(|(name, joint)| JointSpec {
            name,
            id: JointId(joint.id),
            kind: joint.kind.into(),
            topology: JointTopology::Secondary,
            i_endpoint: joint.i_endpoint.into(),
            j_endpoint: joint.j_endpoint.into(),
        });

        primary.chain(secondary).collect()
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlJoint {
    pub id: u16,
    #[serde(rename = "type")]
    pub kind: YamlJointKind,
    #[serde(rename = "i")]
    pub i_endpoint: YamlJointEndpoint,
    #[serde(rename = "j")]
    pub j_endpoint: YamlJointEndpoint,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct YamlJointEndpoint {
    body_id: u16,
    point: String,
    orientation: YamlOrientation,
}

impl From<YamlJointEndpoint> for JointEndpointSpec {
    fn from(value: YamlJointEndpoint) -> Self {
        Self {
            body_id: BodyId(value.body_id),
            point: value.point,
            orientation: value.orientation.into(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlJointKind {
    Revolute,
    Spherical,
    Universal,
    Prismatic,
    Planar,
    Inplane,
    Inline,
}

// Mapping from YamlJointKind to JointKind
impl From<YamlJointKind> for JointKind {
    fn from(value: YamlJointKind) -> Self {
        match value {
            YamlJointKind::Revolute => Self::Revolute,
            YamlJointKind::Spherical => Self::Spherical,
            YamlJointKind::Universal => Self::Universal,
            YamlJointKind::Prismatic => Self::Prismatic,
            YamlJointKind::Planar => Self::Planar,
            YamlJointKind::Inplane => Self::Inplane,
            YamlJointKind::Inline => Self::Inline,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlJointTopology {
    Primary,
    Secondary,
}

// Mapping from YamlJointTopology to JointKind
impl From<YamlJointTopology> for JointTopology {
    fn from(value: YamlJointTopology) -> Self {
        match value {
            YamlJointTopology::Primary => Self::Primary,
            YamlJointTopology::Secondary => Self::Secondary,
        }
    }
}
