use std::collections::BTreeMap;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::Deserialize;

use super::joints::YamlJointTopology;
use crate::kinematics::coordinates::JointCoordinates;
use crate::kinematics::spherical::SphericalCoordinates;
use crate::model::motion::{MotionKind, MotionSpec};
use crate::model::{BodyId, BodyPose, JointId, JointKey};

#[derive(Debug, Clone, PartialEq, Default, Deserialize)]
pub struct YamlMotions(pub BTreeMap<String, YamlMotion>);

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum YamlMotion {
    JointCoordinates {
        joint_topology: YamlJointTopology,
        joint_id: u16,
        relative_orientation: YamlRelativeJointOrientation,
    },
    BodyPose {
        body_id: u16,
        position: [f64; 3],
        orientation: [f64; 4],
    },
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "method", rename_all = "lowercase")]
pub enum YamlRelativeJointOrientation {
    Quaternion { quaternion: [f64; 4] },
}

impl YamlMotion {
    fn into_spec(self, name: String) -> Result<MotionSpec, String> {
        let kind = match self {
            Self::JointCoordinates {
                joint_topology,
                joint_id,
                relative_orientation,
            } => {
                let relative_orientation = relative_orientation
                    .into_quaternion()
                    .ok_or_else(|| format!("invalid quaternion for motion '{name}'"))?;

                MotionKind::JointCoordinates {
                    key: JointKey {
                        topology: joint_topology.into(),
                        id: JointId(joint_id),
                    },
                    coordinates: JointCoordinates::Spherical(SphericalCoordinates {
                        relative_orientation,
                    }),
                }
            }
            Self::BodyPose {
                body_id,
                position,
                orientation,
            } => {
                if position.iter().any(|value| !value.is_finite()) {
                    return Err(format!("non-finite body position for motion '{name}'"));
                }

                let orientation = unit_quaternion(orientation)
                    .ok_or_else(|| format!("invalid body orientation for motion '{name}'"))?;

                MotionKind::BodyPose {
                    body_id: BodyId(body_id),
                    pose: BodyPose::new(Vector3::from(position), orientation),
                }
            }
        };

        Ok(MotionSpec::new(name, kind))
    }
}

impl YamlRelativeJointOrientation {
    fn into_quaternion(self) -> Option<UnitQuaternion<f64>> {
        match self {
            Self::Quaternion { quaternion } => unit_quaternion(quaternion),
        }
    }
}

fn unit_quaternion(values: [f64; 4]) -> Option<UnitQuaternion<f64>> {
    if values.iter().any(|value| !value.is_finite()) {
        return None;
    }

    let [w, x, y, z] = values;
    UnitQuaternion::try_new(Quaternion::new(w, x, y, z), 1.0e-6)
}

impl TryFrom<YamlMotions> for Vec<MotionSpec> {
    type Error = String;

    fn try_from(value: YamlMotions) -> Result<Self, Self::Error> {
        value
            .0
            .into_iter()
            .map(|(name, motion)| motion.into_spec(name))
            .collect()
    }
}
