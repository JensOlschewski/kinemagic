use std::collections::BTreeMap;

use ::serde::Deserialize;

#[derive(Debug, Clone, PartialEq, Default, Deserialize)]
pub struct YamlMotions {
    joint_coordinates: BTreeMap<String, YamlJointCoordinates>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
struct YamlJointCoordinates {
    relative_orientation: YamlRelativeJointOrientation,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag ="method")]
enum YamlRelativeJointOrientation {
    Quaternion { quaternion: [f64; 4] },
}

impl TryFrom <YamlMotions> for BTreeMap<String, [f64; 4]> {
    type Error = String;

    fn try_from(value: YamlMotions) -> Result<Self, Self::Error> {
        value
            .joint_coordinates
            .into_iter()
            .map(|(joint_name, joint_coords)| {
                let quaternion = match joint_coords.relative_orientation {
                    Some(YamlRelativeJointOrientation::Quaternion { quaternion }) => quaternion,
                    None => return Err(format!("Missing quaternion for joint '{}'", joint_name)),
                    };

                Ok((joint_name, quaternion))
            })
            .collect()
    }
}
