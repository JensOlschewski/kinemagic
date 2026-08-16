use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use thiserror::Error;

use crate::model::{
    Bodies, Body, BodyId, Joint, JointId, JointKind, Joints, Marker, Model, ModelBuildError, Point,
};
use nalgebra::{Rotation3, UnitQuaternion, Vector3};

#[derive(Debug, Deserialize)]
pub struct YamlModel {
    pub hardpoints: BTreeMap<String, [f64; 3]>,
    #[serde(default)]
    pub bodies: BTreeMap<String, YamlBody>,
    #[serde(default)]
    pub joints: BTreeMap<String, YamlJoint>,
}

impl YamlModel {
    pub fn into_model(self) -> Result<Model, YamlError> {
        let YamlModel {
            hardpoints,
            bodies,
            joints,
        } = self;

        let body_values = bodies
            .into_iter()
            .map(|(name, body)| body.into_body(&name, &hardpoints))
            .collect::<Result<Vec<Body>, YamlError>>()?;

        let mut body_values = body_values;
        if !body_values.iter().any(|body| body.id() == BodyId::GROUND) {
            body_values.insert(
                0,
                Body::new(
                    BodyId::GROUND,
                    "ground",
                    Vector3::zeros(),
                    UnitQuaternion::identity(),
                    vec![],
                ),
            );
        }

        let joint_values = joints
            .into_iter()
            .map(|(name, joint)| joint.into_joint(&name, &hardpoints))
            .collect::<Result<Vec<Joint>, YamlError>>()?;

        Model::new(Bodies::new(body_values)?, Joints::new(joint_values)?).map_err(Into::into)
    }
}

#[derive(Debug, Deserialize)]
pub struct YamlBody {
    pub body_id: u32,
    pub side: YamlSide,
    pub position: YamlPosition,
    pub orientation: YamlOrientation,
    pub points_on_body: Vec<String>,
}

impl YamlBody {
    pub fn into_body(
        self,
        name: &str,
        hardpoints: &BTreeMap<String, [f64; 3]>,
    ) -> Result<Body, YamlError> {
        let points = self
            .points_on_body
            .into_iter()
            .map(|point_name| {
                let coordinates =
                    hardpoints
                        .get(&point_name)
                        .ok_or_else(|| YamlError::UnknownHardpoint {
                            body: name.to_owned(),
                            point: point_name.clone(),
                        })?;

                let [x, y, z] = *coordinates;

                Ok(Point::new(point_name, Vector3::new(x, y, z)))
            })
            .collect::<Result<Vec<_>, YamlError>>()?;

        let position =
            self.position
                .into_vector(hardpoints)
                .map_err(|point| YamlError::UnknownHardpoint {
                    body: name.to_owned(),
                    point,
                })?;

        let orientation = match self.orientation {
            YamlOrientation::Euler { euler_angles } => {
                let [roll, pitch, yaw] = euler_angles;
                UnitQuaternion::from_euler_angles(roll, pitch, yaw)
            }
        };

        Ok(Body::new(
            BodyId::new(self.body_id),
            name,
            position,
            orientation,
            points,
        ))
    }
}

#[derive(Debug, Deserialize)]
pub struct YamlJoint {
    pub joint_id: u32,
    pub kind: YamlJointKind,
    pub i: YamlMarker,
    pub j: YamlMarker,
}

impl YamlJoint {
    pub fn into_joint(
        self,
        name: &str,
        hardpoints: &BTreeMap<String, [f64; 3]>,
    ) -> Result<Joint, YamlError> {
        let i_name = format!("{name}:i");
        let j_name = format!("{name}:j");
        let i_marker = self.i.into_marker(&i_name, hardpoints)?;
        let j_marker = self.j.into_marker(&j_name, hardpoints)?;

        Ok(Joint::new(
            JointId::new(self.joint_id),
            name,
            self.kind.into_joint_kind(),
            i_marker,
            j_marker,
        ))
    }
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlSide {
    Single,
    Left,
    Right,
}

#[derive(Debug, Deserialize)]
pub struct YamlMarker {
    pub body_id: u32,
    pub position: YamlPosition,
    pub orientation: YamlOrientation,
}

impl YamlMarker {
    pub fn into_marker(
        self,
        name: &str,
        hardpoints: &BTreeMap<String, [f64; 3]>,
    ) -> Result<Marker, YamlError> {
        let position = self.position.into_vector(hardpoints).map_err(|point| {
            YamlError::UnknownMarkerHardpoint {
                marker: name.to_owned(),
                point,
            }
        })?;
        let [roll, pitch, yaw] = self.orientation.into_euler_angles();

        Ok(Marker::new(
            name,
            BodyId::new(self.body_id),
            position,
            Rotation3::from_euler_angles(roll, pitch, yaw),
        ))
    }
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlJointKind {
    Spherical,
}

impl YamlJointKind {
    pub fn into_joint_kind(self) -> JointKind {
        match self {
            YamlJointKind::Spherical => JointKind::Spherical,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "method")]
pub enum YamlOrientation {
    #[serde(rename = "euler")]
    Euler { euler_angles: [f64; 3] },
}

impl YamlOrientation {
    fn into_euler_angles(self) -> [f64; 3] {
        match self {
            Self::Euler { euler_angles } => euler_angles,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
pub enum YamlPosition {
    Hardpoint(String),
    Coordinates([f64; 3]),
}

impl YamlPosition {
    fn into_vector(self, hardpoints: &BTreeMap<String, [f64; 3]>) -> Result<Vector3<f64>, String> {
        let coordinates = match self {
            Self::Hardpoint(name) => hardpoints.get(&name).copied().ok_or(name)?,
            Self::Coordinates(coordinates) => coordinates,
        };
        let [x, y, z] = coordinates;

        Ok(Vector3::new(x, y, z))
    }
}

#[derive(Debug, Error)]
pub enum YamlError {
    #[error("invalid YAML: {0}")]
    Parse(#[source] serde_yaml_ng::Error),
    #[error("failed to read `{path}`: {source}")]
    Read {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("body `{body}` references unknown hardpoint `{point}`")]
    UnknownHardpoint { body: String, point: String },
    #[error("marker `{marker}` references unknown hardpoint `{point}`")]
    UnknownMarkerHardpoint { marker: String, point: String },
    #[error(transparent)]
    Model(#[from] ModelBuildError),
}

pub fn parse_yaml_str(input: &str) -> Result<YamlModel, YamlError> {
    serde_yaml_ng::from_str(input).map_err(YamlError::Parse)
}

pub fn parse_yaml_file(path: impl AsRef<Path>) -> Result<YamlModel, YamlError> {
    let path = path.as_ref();

    let input = std::fs::read_to_string(path).map_err(|source| YamlError::Read {
        path: path.to_owned(),
        source,
    })?;

    parse_yaml_str(&input)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_example_model() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml");

        let model = parse_yaml_str(input)?;

        assert_eq!(model.hardpoints.len(), 1);
        assert_eq!(model.bodies.len(), 1);
        assert_eq!(model.joints.len(), 1);
        assert_eq!(model.bodies["B1"].body_id, 1);
        assert_eq!(model.joints["J1"].joint_id, 1);

        Ok(())
    }

    #[test]
    fn converts_example_into_model() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml");

        let model = parse_yaml_str(input)?.into_model()?;

        assert_eq!(model.bodies().iter().count(), 2);
        assert_eq!(model.bodies().get(BodyId::GROUND).unwrap().name(), "ground");
        let body = model.bodies().get(BodyId::new(1)).unwrap();
        assert_eq!(body.name(), "B1");
        assert_eq!(body.position(), Vector3::new(0.0, 0.0, -100.0));
        assert_eq!(body.points().len(), 1);
        assert_eq!(body.points()[0].name(), "P1");
        assert_eq!(body.points()[0].position(), Vector3::zeros());

        let joint = model.joints().iter().next().unwrap();
        assert_eq!(joint.id(), JointId::new(1));
        assert_eq!(joint.name(), "J1");
        assert_eq!(joint.i_marker().name(), "J1:i");
        assert_eq!(joint.i_marker().body_id(), BodyId::GROUND);
        assert_eq!(joint.j_marker().name(), "J1:j");
        assert_eq!(joint.j_marker().body_id(), BodyId::new(1));
        assert_eq!(joint.i_marker().position(), Vector3::zeros());
        assert_eq!(joint.j_marker().position(), Vector3::zeros());

        Ok(())
    }

    #[test]
    fn rejects_unknown_marker_hardpoint() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml").replacen(
            "position: P1",
            "position: missing",
            1,
        );

        let result = parse_yaml_str(&input)?.into_model();

        assert!(matches!(
            result,
            Err(YamlError::UnknownMarkerHardpoint { marker, point })
                if marker == "J1:i" && point == "missing"
        ));

        Ok(())
    }

    #[test]
    fn rejects_unknown_body_hardpoint() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml")
            .replace("points_on_body: [P1]", "points_on_body: [missing]");

        let result = parse_yaml_str(&input)?.into_model();

        assert!(matches!(
            result,
            Err(YamlError::UnknownHardpoint { body, point })
                if body == "B1" && point == "missing"
        ));

        Ok(())
    }

    #[test]
    fn converts_inline_marker_position() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml").replacen(
            "position: P1",
            "position: [1.0, 2.0, 3.0]",
            1,
        );

        let model = parse_yaml_str(&input)?.into_model()?;
        let joint = model.joints().iter().next().unwrap();

        assert_eq!(joint.i_marker().position(), Vector3::new(1.0, 2.0, 3.0));

        Ok(())
    }

    #[test]
    fn parses_yaml_file() -> Result<(), YamlError> {
        let model = parse_yaml_file("tests/fixtures/spherical_one_body_parse.yaml")?;

        assert_eq!(model.bodies.len(), 1);
        assert_eq!(model.joints.len(), 1);

        Ok(())
    }

    #[test]
    fn rejects_missing_yaml_file() {
        let result = parse_yaml_file("tests/fixtures/missing-model.yaml");

        assert!(matches!(result, Err(YamlError::Read { .. })));
    }

    #[test]
    fn rejects_malformed_yaml() {
        let result = parse_yaml_str("bodies: [");

        assert!(matches!(result, Err(YamlError::Parse(_))));
    }
}
