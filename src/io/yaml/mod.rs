use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use thiserror::Error;

use crate::model::{
    Bodies, Body, BodyId, Input, Joint, JointId, JointKind, Joints, Marker, Model, ModelBuildError,
    Motion, MotionKind, Point,
};
use nalgebra::{UnitQuaternion, Vector3};

#[derive(Debug, Deserialize)]
pub struct YamlInput {
    pub hardpoints: BTreeMap<String, [f64; 3]>,
    #[serde(default)]
    pub bodies: BTreeMap<String, YamlBody>,
    #[serde(default)]
    pub joints: BTreeMap<String, YamlJoint>,
    #[serde(default)]
    pub motions: BTreeMap<String, YamlMotion>,
}

impl YamlInput {
    pub fn into_input(self) -> Result<Input, YamlError> {
        let YamlInput {
            hardpoints,
            bodies,
            joints,
            motions,
        } = self;

        let body_values = bodies
            .into_iter()
            .map(|(name, body)| body.into_body(&name, &hardpoints))
            .collect::<Result<Vec<Body>, YamlError>>()?;

        let mut body_values = body_values;

        if let Some(ground) = body_values.iter().find(|body| body.id() == BodyId::GROUND) {
            return Err(YamlError::ExplicitGround {
                name: ground.name().to_owned(),
            });
        }

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

        let bodies = Bodies::new(body_values)?;

        let joint_values = joints
            .into_iter()
            .map(|(name, joint)| joint.into_joint(&name, &bodies, &hardpoints))
            .collect::<Result<Vec<Joint>, YamlError>>()?;

        let model = Model::new(bodies, Joints::new(joint_values)?)?;
        let motions = motions
            .into_iter()
            .map(|(name, motion)| motion.into_motion(name))
            .collect();

        Ok(Input::new(model, motions))
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
        let position =
            self.position
                .into_vector(hardpoints)
                .map_err(|point| YamlError::UnknownHardpoint {
                    name: name.to_owned(),
                    point,
                })?;

        let orientation = self.orientation.into_unit_quaternion();

        let local_points = self
            .points_on_body
            .into_iter()
            .map(|point_name| {
                let coordinates =
                    hardpoints
                        .get(&point_name)
                        .ok_or_else(|| YamlError::UnknownHardpoint {
                            name: name.to_owned(),
                            point: point_name.clone(),
                        })?;

                let [x, y, z] = *coordinates;

                let point = Vector3::new(x, y, z);

                let local_point = orientation.inverse_transform_vector(&(point - position));

                Ok(Point::new(point_name, local_point))
            })
            .collect::<Result<Vec<_>, YamlError>>()?;

        Ok(Body::new(
            BodyId::new(self.body_id),
            name,
            position,
            orientation,
            local_points,
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
        bodies: &Bodies,
        hardpoints: &BTreeMap<String, [f64; 3]>,
    ) -> Result<Joint, YamlError> {
        let i_body =
            bodies
                .get(BodyId::new(self.i.body_id))
                .ok_or_else(|| YamlError::UnknownJointBody {
                    name: format!("{name}:i"),
                    body_id: self.i.body_id,
                })?;

        let j_body =
            bodies
                .get(BodyId::new(self.j.body_id))
                .ok_or_else(|| YamlError::UnknownJointBody {
                    name: format!("{name}:j"),
                    body_id: self.j.body_id,
                })?;

        let i_name = format!("{name}:i");
        let j_name = format!("{name}:j");
        let i_marker = self.i.into_marker(&i_name, i_body, hardpoints)?;
        let j_marker = self.j.into_marker(&j_name, j_body, hardpoints)?;

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
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum YamlMotion {
    JointCoordinates {
        joint_id: u32,
        relative_orientation: YamlOrientation,
    },
}

impl YamlMotion {
    fn into_motion(self, name: String) -> Motion {
        match self {
            Self::JointCoordinates {
                joint_id,
                relative_orientation,
            } => Motion::new(
                name,
                MotionKind::JointCoordinates,
                JointId::new(joint_id),
                relative_orientation.into_unit_quaternion(),
            ),
        }
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
        body: &Body,
        hardpoints: &BTreeMap<String, [f64; 3]>,
    ) -> Result<Marker, YamlError> {
        let local_orientation =
            body.orientation().inverse() * self.orientation.into_unit_quaternion();

        let position = self.position.into_vector(hardpoints).map_err(|point| {
            YamlError::UnknownMarkerHardpoint {
                name: name.to_owned(),
                point,
            }
        })?;

        let local_position = body.orientation().inverse() * (position - body.position());

        Ok(Marker::new(
            name,
            BodyId::new(self.body_id),
            local_position,
            local_orientation,
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
    fn into_unit_quaternion(self) -> UnitQuaternion<f64> {
        match self {
            Self::Euler { euler_angles } => {
                let [alpha, beta, gamma] = euler_angles.map(f64::to_radians);

                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), alpha)
                    * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), beta)
                    * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), gamma)
            }
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
    #[error("body `{name}` references unknown hardpoint `{point}`")]
    UnknownHardpoint { name: String, point: String },
    #[error("marker `{name}` references unknown hardpoint `{point}`")]
    UnknownMarkerHardpoint { name: String, point: String },
    #[error("marker `{name}` references unknown body '{body_id}`")]
    UnknownJointBody { name: String, body_id: u32 },
    #[error("body `{name}` uses reserved body ID 0; ground is implicit")]
    ExplicitGround { name: String },
    #[error(transparent)]
    Model(#[from] ModelBuildError),
}

pub fn parse_yaml_str(input: &str) -> Result<YamlInput, YamlError> {
    serde_yaml_ng::from_str(input).map_err(YamlError::Parse)
}

pub fn parse_yaml_file(path: impl AsRef<Path>) -> Result<YamlInput, YamlError> {
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
    fn converts_spherical_one_body_parse_into_model() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml");

        let input = parse_yaml_str(input)?.into_input()?;
        let model = input.model();

        assert_eq!(model.bodies().iter().count(), 2);
        assert_eq!(model.bodies().get(BodyId::GROUND).unwrap().name(), "ground");
        let body = model.bodies().get(BodyId::new(1)).unwrap();
        assert_eq!(body.name(), "B1");
        assert_eq!(body.position(), Vector3::new(0.0, 0.0, -100.0));
        assert_eq!(body.points().len(), 1);
        assert_eq!(body.points()[0].name(), "P1");
        assert_eq!(body.points()[0].position(), Vector3::new(0.0, 0.0, 100.0));

        let joint = model.joints().iter().next().unwrap();
        assert_eq!(joint.id(), JointId::new(1));
        assert_eq!(joint.name(), "J1");
        assert_eq!(joint.i_marker().name(), "J1:i");
        assert_eq!(joint.i_marker().body_id(), BodyId::GROUND);
        assert_eq!(joint.j_marker().name(), "J1:j");
        assert_eq!(joint.j_marker().body_id(), BodyId::new(1));
        assert_eq!(joint.i_marker().position(), Vector3::zeros());
        assert_eq!(joint.j_marker().position(), Vector3::new(0.0, 0.0, 100.0));

        Ok(())
    }

    #[test]
    fn converts_spherical_two_body_parse_into_model() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_two_body_parse.yaml");

        let input = parse_yaml_str(input)?.into_input()?;
        let model = input.model();

        assert_eq!(model.bodies().iter().count(), 3);
        assert_eq!(model.joints().iter().count(), 2);
        assert_eq!(model.bodies().get(BodyId::GROUND).unwrap().name(), "ground");

        let body_1 = model.bodies().get(BodyId::new(1)).unwrap();
        assert_eq!(body_1.points().len(), 2);
        assert_eq!(body_1.points()[0].position(), Vector3::new(0.0, 0.0, 100.0));
        assert_eq!(
            body_1.points()[1].position(),
            Vector3::new(0.0, 0.0, -100.0)
        );

        let body_2 = model.bodies().get(BodyId::new(2)).unwrap();
        assert_eq!(body_2.points().len(), 1);
        assert_eq!(body_2.points()[0].position(), Vector3::new(0.0, 0.0, 100.0));

        let mut joints = model.joints().iter();
        let joint_1 = joints.next().unwrap();
        assert_eq!(joint_1.id(), JointId::new(1));
        assert_eq!(joint_1.i_marker().position(), Vector3::zeros());
        assert_eq!(joint_1.j_marker().position(), Vector3::new(0.0, 0.0, 100.0));

        let joint_2 = joints.next().unwrap();
        assert_eq!(joint_2.id(), JointId::new(2));
        assert_eq!(
            joint_2.i_marker().position(),
            Vector3::new(0.0, 0.0, -100.0)
        );
        assert_eq!(joint_2.j_marker().position(), Vector3::new(0.0, 0.0, 100.0));

        Ok(())
    }

    #[test]
    fn converts_missing_motions_into_empty_input() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml");

        let input = parse_yaml_str(input)?.into_input()?;

        assert!(input.motions().is_empty());

        Ok(())
    }

    #[test]
    fn converts_joint_coordinate_motion_into_input() -> Result<(), YamlError> {
        let yaml = format!(
            "{}\nmotions:\n  RotateJ1:\n    kind: joint-coordinates\n    joint_id: 1\n    relative_orientation:\n      method: euler\n      euler_angles: [0, 90, 0]\n",
            include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml")
        );

        let input = parse_yaml_str(&yaml)?.into_input()?;
        let motion = &input.motions()[0];
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);

        assert_eq!(motion.name(), "RotateJ1");
        assert_eq!(motion.joint_id(), JointId::new(1));
        assert!(motion.relative_orientation().angle_to(&expected) < 1.0e-12);

        Ok(())
    }

    #[test]
    fn rejects_unknown_motion_kind() {
        let yaml = format!(
            "{}\nmotions:\n  RotateJ1:\n    kind: unsupported\n    joint_id: 1\n    relative_orientation:\n      method: euler\n      euler_angles: [0, 90, 0]\n",
            include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml")
        );

        assert!(matches!(parse_yaml_str(&yaml), Err(YamlError::Parse(_))));
    }

    #[test]
    fn converts_rotated_body_geometry_to_local_frame() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml")
            .replace("P1: [0.0, 0.0, 0.0]", "P1: [1.0, 0.0, 0.0]")
            .replace("position: [0.0, 0.0, -100.0]", "position: [0.0, 0.0, 0.0]")
            .replacen("euler_angles: [0, 0, 0]", "euler_angles: [90, 0, 0]", 1);

        let input = parse_yaml_str(&input)?.into_input()?;
        let model = input.model();
        let body = model.bodies().get(BodyId::new(1)).unwrap();
        let joint = model.joints().iter().next().unwrap();
        let expected_world_point = Vector3::new(1.0, 0.0, 0.0);

        let body_point_world = body.position()
            + body
                .orientation()
                .transform_vector(&body.points()[0].position());
        assert!((body_point_world - expected_world_point).norm() < 1.0e-12);

        let marker_world_position = body.position()
            + body
                .orientation()
                .transform_vector(&joint.j_marker().position());
        assert!((marker_world_position - expected_world_point).norm() < 1.0e-12);

        let marker_world_orientation = body.orientation() * joint.j_marker().orientation();
        assert!(marker_world_orientation.angle() < 1.0e-12);

        Ok(())
    }

    #[test]
    fn rejects_unknown_marker_hardpoint() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml").replacen(
            "position: P1",
            "position: missing",
            1,
        );

        let result = parse_yaml_str(&input)?.into_input();

        assert!(matches!(
            result,
            Err(YamlError::UnknownMarkerHardpoint { name, point })
                if name == "J1:i" && point == "missing"
        ));

        Ok(())
    }

    #[test]
    fn rejects_unknown_body_hardpoint() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml")
            .replace("points_on_body: [P1]", "points_on_body: [missing]");

        let result = parse_yaml_str(&input)?.into_input();

        assert!(matches!(
            result,
            Err(YamlError::UnknownHardpoint { name, point })
                if name == "B1" && point == "missing"
        ));

        Ok(())
    }

    #[test]
    fn rejects_unknown_marker_body() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml").replacen(
            "body_id: 0",
            "body_id: 99",
            1,
        );

        let result = parse_yaml_str(&input)?.into_input();

        assert!(matches!(
            result,
            Err(YamlError::UnknownJointBody { name, body_id }) if name == "J1:i" && body_id == 99
        ));

        Ok(())
    }

    #[test]
    fn rejects_explicit_ground_body() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml").replacen(
            "body_id: 1",
            "body_id: 0",
            1,
        );

        let result = parse_yaml_str(&input)?.into_input();

        assert!(matches!(
            result,
            Err(YamlError::ExplicitGround { name }) if name == "B1"
        ));

        Ok(())
    }

    #[test]
    fn rejects_missing_fields_malformed_vectors_and_invalid_enums() {
        let valid = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml");
        let invalid_inputs = [
            valid.replace("    side: single\n", ""),
            valid.replace("position: [0.0, 0.0, -100.0]", "position: [0.0, 0.0]"),
            valid.replace("kind: spherical", "kind: revolute"),
        ];

        for input in invalid_inputs {
            assert!(matches!(parse_yaml_str(&input), Err(YamlError::Parse(_))));
        }
    }

    #[test]
    fn converts_inline_marker_position() -> Result<(), YamlError> {
        let input = include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml").replacen(
            "position: P1",
            "position: [1.0, 2.0, 3.0]",
            1,
        );

        let input = parse_yaml_str(&input)?.into_input()?;
        let model = input.model();
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
