use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

use crate::model::{
    Bodies, Body, BodyId, Input, Joint, JointDisplacement, JointId, JointKind, JointRole, Joints,
    Marker, Model, ModelBuildError, Motion, MotionKind, Point,
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
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
        self.validate()?;

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

    fn validate(&self) -> Result<(), YamlError> {
        for (name, hardpoint) in &self.hardpoints {
            validate_components(&format!("hardpoints.{name}"), hardpoint)?;
        }

        for (name, body) in &self.bodies {
            body.position
                .validate_finite(&format!("bodies.{name}.position"))?;

            body.orientation
                .validate_finite(&format!("bodies.{name}.orientation.euler_angles"))?;
        }

        for (name, joint) in &self.joints {
            joint
                .i
                .position
                .validate_finite(&format!("joints.{name}.i.position"))?;

            joint
                .i
                .orientation
                .validate_finite(&format!("joints.{name}.i.orientation.euler_angles"))?;

            joint
                .j
                .position
                .validate_finite(&format!("joints.{name}.j.position"))?;

            joint
                .j
                .orientation
                .validate_finite(&format!("joints.{name}.j.orientation.euler_angles"))?;
        }

        for (name, motion) in &self.motions {
            motion.validate(name, &format!("motions.{name}.displacement"))?;
        }

        Ok(())
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
pub struct YamlJoint {
    pub joint_id: u32,
    pub kind: YamlJointKind,
    #[serde(default)]
    pub role: YamlJointRole,
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
            self.role.into_joint_role(),
            i_marker,
            j_marker,
        ))
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct YamlMotion {
    kind: YamlMotionKind,
    joint_id: u32,
    displacement: BTreeMap<YamlJointDisplacement, f64>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum YamlMotionKind {
    JointCoordinates,
}

impl YamlMotion {
    fn into_motion(self, name: String) -> Motion {
        let mut rotation = Vector3::new(None, None, None);

        for (component, degrees) in self.displacement {
            let radians = Some(degrees.to_radians());

            match component {
                YamlJointDisplacement::RotX => rotation.x = radians,
                YamlJointDisplacement::RotY => rotation.y = radians,
                YamlJointDisplacement::RotZ => rotation.z = radians,
            }
        }

        match self.kind {
            YamlMotionKind::JointCoordinates => Motion::new(
                name,
                MotionKind::JointCoordinates,
                JointId::new(self.joint_id),
                JointDisplacement::new(rotation),
            ),
        }
    }

    fn validate(&self, motion_name: &str, path: &str) -> Result<(), YamlError> {
        if self.displacement.is_empty() {
            return Err(YamlError::EmptyMotionDisplacement {
                name: motion_name.to_owned(),
            });
        }

        for (component, value) in &self.displacement {
            let component_name = match component {
                YamlJointDisplacement::RotX => "rot_x",
                YamlJointDisplacement::RotY => "rot_y",
                YamlJointDisplacement::RotZ => "rot_z",
            };
            let component_path = format!("{path}.{component_name}");
            validate_finite_value(&component_path, *value)?;
        }

        Ok(())
    }
}

#[derive(Debug, Deserialize, Ord, PartialOrd, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
enum YamlJointDisplacement {
    RotX,
    RotY,
    RotZ,
}

#[derive(Debug, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlJointRole {
    #[default]
    Auto,
    Primary,
    Secondary,
}

impl YamlJointRole {
    pub fn into_joint_role(self) -> JointRole {
        match self {
            YamlJointRole::Auto => JointRole::Auto,
            YamlJointRole::Primary => JointRole::Primary,
            YamlJointRole::Secondary => JointRole::Secondary,
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
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
pub struct YamlOrientation {
    method: YamlOrientationMethod,
    euler_angles: [f64; 3],
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "lowercase")]
enum YamlOrientationMethod {
    Euler,
}

impl YamlOrientation {
    fn into_unit_quaternion(self) -> UnitQuaternion<f64> {
        match self.method {
            YamlOrientationMethod::Euler => {
                let euler_angles = self.euler_angles;
                let [alpha, beta, gamma] = euler_angles.map(f64::to_radians);

                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), alpha)
                    * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), beta)
                    * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), gamma)
            }
        }
    }

    fn validate_finite(&self, path: &str) -> Result<(), YamlError> {
        validate_components(path, &self.euler_angles)
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

    fn validate_finite(&self, path: &str) -> Result<(), YamlError> {
        match self {
            Self::Hardpoint(_) => Ok(()),
            Self::Coordinates(coordinates) => validate_components(path, coordinates),
        }
    }
}

// Helper function to validate scalar values for finiteness
// returning a YamlError if the value is not finite.
fn validate_finite_value(path: &str, value: f64) -> Result<(), YamlError> {
    if !value.is_finite() {
        return Err(YamlError::NonFinite {
            path: path.to_owned(),
            value,
        });
    }

    Ok(())
}

// Helper function to validate that all components of a 3D vector are
// finite numbers
fn validate_components(path: &str, values: &[f64; 3]) -> Result<(), YamlError> {
    for (index, value) in values.iter().copied().enumerate() {
        validate_finite_value(&format!("{path}[{index}]"), value)?;
    }

    Ok(())
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
    #[error("motion `{name}` has empty displacement: {{}}")]
    EmptyMotionDisplacement { name: String },
    #[error("non-finite value `{value}` at `{path}`")]
    NonFinite { path: String, value: f64 },
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

    const VALID_ONE_BODY_PARSE_INPUT: &str =
        include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml");
    const VALID_TWO_BODY_PARSE_INPUT: &str =
        include_str!("../../../tests/fixtures/spherical_two_body_parse.yaml");

    #[test]
    fn converts_spherical_one_body_parse_into_model() -> Result<(), YamlError> {
        let input = parse_yaml_str(VALID_ONE_BODY_PARSE_INPUT)?.into_input()?;
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
        let input = parse_yaml_str(VALID_TWO_BODY_PARSE_INPUT)?.into_input()?;

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
        let input = parse_yaml_str(VALID_ONE_BODY_PARSE_INPUT)?.into_input()?;

        assert!(input.motions().is_empty());

        Ok(())
    }

    #[test]
    fn converts_joint_coordinate_motion_into_input() -> Result<(), YamlError> {
        let requested = Vector3::new(
            Some(std::f64::consts::FRAC_PI_2),
            Some(std::f64::consts::FRAC_PI_2),
            None,
        );

        let yaml = format!(
            "{}\nmotions:\n  RotateJ1:\n    kind: joint-coordinates\n    joint_id: 1\n    displacement:\n      rot_x: 90.0\n      rot_y: 90.0\n",
            include_str!("../../../tests/fixtures/spherical_one_body_parse.yaml")
        );

        let input = parse_yaml_str(&yaml)?.into_input()?;
        let motion = &input.motions()[0];
        let actual = motion.joint_displacement().rotation();

        assert_eq!(motion.name(), "RotateJ1");
        assert_eq!(motion.joint_id(), JointId::new(1));
        assert_eq!(actual, &requested);

        Ok(())
    }

    #[test]
    fn rejects_unknown_fields_with_context() {
        let valid = VALID_ONE_BODY_PARSE_INPUT;
        let motion = include_str!("../../../tests/fixtures/spherical_two_body_motion.yaml");
        let cases = [
            (
                format!("{valid}\nunexpected: true\n"),
                "unexpected",
                "unexpected",
            ),
            (
                valid.replacen(
                    "    body_id: 1\n",
                    "    body_id: 1\n    unexpected: true\n",
                    1,
                ),
                "bodies.B1",
                "unexpected",
            ),
            (
                valid.replace(
                    "    joint_id: 1\n",
                    "    joint_id: 1\n    unexpected: true\n",
                ),
                "joints.J1",
                "unexpected",
            ),
            (
                valid.replace(
                    "      body_id: 0\n",
                    "      body_id: 0\n      unexpected: true\n",
                ),
                "joints.J1.i",
                "unexpected",
            ),
            (
                valid.replacen(
                    "      method: euler\n",
                    "      method: euler\n      unexpected: true\n",
                    1,
                ),
                "bodies.B1.orientation",
                "unexpected",
            ),
            (
                motion.replace(
                    "  RotateJ1:\n    kind: joint-coordinates\n",
                    "  RotateJ1:\n    kind: joint-coordinates\n    unexpected: true\n",
                ),
                "motions.RotateJ1",
                "unexpected",
            ),
            (
                motion.replacen("rot_x: 90", "rot_q: 90", 1),
                "motions.RotateJ1.displacement",
                "rot_q",
            ),
        ];

        for (yaml, context, supplied) in cases {
            let error = parse_yaml_str(&yaml).unwrap_err().to_string();

            assert!(error.contains(supplied), "{context}: {error}");
            assert!(error.contains(context), "{context}: {error}");
        }
    }

    #[test]
    fn rejects_joint_nesting() {
        let input = VALID_ONE_BODY_PARSE_INPUT.replace("  J1:", "  primary:");

        for section in ["primary", "secondary"] {
            let yaml = format!("hardpoints: {{}}\njoints:\n  {section}:\n    J1: {{}}\n");

            assert!(matches!(parse_yaml_str(&yaml), Err(YamlError::Parse(_))));
        }

        assert!(parse_yaml_str(&input).is_ok());
    }

    #[test]
    fn rejects_unsupported_features_with_context() {
        let valid = VALID_ONE_BODY_PARSE_INPUT;
        let motion = include_str!("../../../tests/fixtures/spherical_two_body_motion.yaml");
        let cases = [
            (
                valid.replace("kind: spherical", "kind: revolute"),
                "J1",
                "revolute",
            ),
            (
                motion.replacen("kind: joint-coordinates", "kind: unsupported", 1),
                "RotateJ1",
                "unsupported",
            ),
            (
                valid.replacen("method: euler", "method: quaternion", 1),
                "B1",
                "quaternion",
            ),
        ];

        for (yaml, context, supplied) in cases {
            let error = parse_yaml_str(&yaml).unwrap_err().to_string();

            assert!(error.contains(context), "{error}");
            assert!(error.contains(supplied), "{error}");
        }
    }

    #[test]
    fn rejects_non_finite_components() {
        let valid = VALID_ONE_BODY_PARSE_INPUT;
        let motion = format!(
            "{valid}\nmotions:\n  RotateJ1:\n    kind: joint-coordinates\n    joint_id: 1\n    displacement:\n      rot_x: .nan\n"
        );
        let cases = [
            (
                valid.replace("P1: [0.0, 0.0, 0.0]", "P1: [0.0, .nan, 0.0]"),
                "hardpoints.P1[1]",
            ),
            (
                valid.replace(
                    "position: [0.0, 0.0, -100.0]",
                    "position: [.inf, 0.0, -100.0]",
                ),
                "bodies.B1.position[0]",
            ),
            (
                valid.replacen("position: P1", "position: [0.0, -.inf, 0.0]", 1),
                "joints.J1.i.position[1]",
            ),
            (
                valid.replacen("euler_angles: [0, 0, 0]", "euler_angles: [0, .nan, 0]", 1),
                "bodies.B1.orientation.euler_angles[1]",
            ),
            (
                valid.replacen(
                    "        euler_angles: [0, 0, 0]",
                    "        euler_angles: [0, 0, .inf]",
                    1,
                ),
                "joints.J1.i.orientation.euler_angles[2]",
            ),
            (motion, "motions.RotateJ1.displacement.rot_x"),
        ];

        for (yaml, expected_path) in cases {
            let result = parse_yaml_str(&yaml).unwrap().into_input();
            let error = match result {
                Ok(_) => panic!("non-finite input should fail at {expected_path}"),
                Err(error) => error,
            };

            assert!(matches!(
                error,
                YamlError::NonFinite { path, .. } if path == expected_path
            ));
        }
    }

    #[test]
    fn rejects_explicit_empty_motion_displacement() {
        let valid = VALID_ONE_BODY_PARSE_INPUT;
        let yaml = format!(
            "{valid}\nmotions:\n  RotateJ1:\n    kind: joint-coordinates\n    joint_id: 1\n    displacement: {{}}\n"
        );

        let result = parse_yaml_str(&yaml).unwrap().into_input();

        assert!(matches!(
            result,
            Err(YamlError::EmptyMotionDisplacement { name }) if name == "RotateJ1"
        ));
    }

    #[test]
    fn converts_rotated_body_geometry_to_local_frame() -> Result<(), YamlError> {
        let input = VALID_ONE_BODY_PARSE_INPUT
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
        let input = VALID_ONE_BODY_PARSE_INPUT.replacen("position: P1", "position: missing", 1);

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
        let input =
            VALID_ONE_BODY_PARSE_INPUT.replace("points_on_body: [P1]", "points_on_body: [missing]");

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
        let input = VALID_ONE_BODY_PARSE_INPUT.replacen("body_id: 0", "body_id: 99", 1);

        let result = parse_yaml_str(&input)?.into_input();

        assert!(matches!(
            result,
            Err(YamlError::UnknownJointBody { name, body_id }) if name == "J1:i" && body_id == 99
        ));

        Ok(())
    }

    #[test]
    fn rejects_explicit_ground_body() -> Result<(), YamlError> {
        let input = VALID_ONE_BODY_PARSE_INPUT.replacen("body_id: 1", "body_id: 0", 1);

        let result = parse_yaml_str(&input)?.into_input();

        assert!(matches!(
            result,
            Err(YamlError::ExplicitGround { name }) if name == "B1"
        ));

        Ok(())
    }

    #[test]
    fn rejects_missing_fields_malformed_vectors_and_invalid_enums() {
        let valid = VALID_ONE_BODY_PARSE_INPUT;
        let invalid_inputs = [
            valid.replace("    side: single\n", ""),
            valid.replace("position: [0.0, 0.0, -100.0]", "position: [0.0, 0.0]"),
        ];

        for input in invalid_inputs {
            assert!(matches!(parse_yaml_str(&input), Err(YamlError::Parse(_))));
        }
    }

    #[test]
    fn rejects_unknown_joint_role() {
        let valid = VALID_ONE_BODY_PARSE_INPUT;

        let yaml = valid.replacen(
            "    kind: spherical\n",
            "    kind: spherical\n    role: invalid\n",
            1,
        );

        let error = parse_yaml_str(&yaml).unwrap_err();

        assert!(matches!(error, YamlError::Parse(_)));

        let message = error.to_string();

        assert!(message.contains("joints.J1"), "{message}");
        assert!(message.contains("invalid"), "{message}");
    }

    #[test]
    fn converts_joint_roles() -> Result<(), YamlError> {
        let cases = [
            (None, JointRole::Auto),
            (Some("auto"), JointRole::Auto),
            (Some("primary"), JointRole::Primary),
            (Some("secondary"), JointRole::Secondary),
        ];

        for (role, expected) in cases {
            let yaml = match role {
                None => VALID_ONE_BODY_PARSE_INPUT.to_owned(),
                Some(role) => VALID_ONE_BODY_PARSE_INPUT.replacen(
                    "    kind: spherical\n",
                    &format!("    kind: spherical\n    role: {role}\n"),
                    1,
                ),
            };

            let input = parse_yaml_str(&yaml)?.into_input()?;
            let actual = input.model().joints().iter().next().unwrap().role();

            assert_eq!(actual, expected);
        }

        Ok(())
    }

    #[test]
    fn converts_inline_marker_position() -> Result<(), YamlError> {
        let valid =
            VALID_ONE_BODY_PARSE_INPUT.replacen("position: P1", "position: [1.0, 2.0, 3.0]", 1);

        let input = parse_yaml_str(&valid)?.into_input()?;
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
