use crate::model::{Input, JointId};
use nalgebra::UnitQuaternion;
use std::collections::BTreeMap;
use thiserror::Error;

const REFERENCE_POSITION_TOLERANCE: f64 = 1.0e-9;

#[derive(Debug)]
pub struct JointCoordinates {
    values: BTreeMap<JointId, JointCoordinate>,
}

impl JointCoordinates {
    pub fn get(&self, joint_id: JointId) -> Option<&JointCoordinate> {
        self.values.get(&joint_id)
    }

    pub fn iter(&self) -> impl Iterator<Item = (JointId, &JointCoordinate)> {
        self.values
            .iter()
            .map(|(&joint_id, coordinate)| (joint_id, coordinate))
    }
}

#[derive(Debug)]
pub struct JointCoordinate {
    relative_orientation: UnitQuaternion<f64>,
}

impl JointCoordinate {
    pub fn new(relative_orientation: UnitQuaternion<f64>) -> Self {
        Self {
            relative_orientation,
        }
    }

    pub fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.relative_orientation
    }
}

pub fn resolve_joint_coordinates(input: &Input) -> Result<JointCoordinates, JointCoordinateError> {
    let mut motions_by_joint = BTreeMap::new();
    let joints = input.model().joints();

    for motion in input.motions() {
        let joint_id = motion.joint_id();

        if !joints.contains(joint_id) {
            return Err(JointCoordinateError::UnknownJoint {
                motion_name: motion.name().to_owned(),
                joint_id,
            });
        }

        if let Some(first_motion) = motions_by_joint.insert(joint_id, motion) {
            return Err(JointCoordinateError::DuplicateMotion {
                joint_id,
                first_motion: first_motion.name().to_owned(),
                second_motion: motion.name().to_owned(),
            });
        }
    }

    let mut values = BTreeMap::new();

    for joint in joints.iter() {
        let relative_orientation = if let Some(motion) = motions_by_joint.get(&joint.id()) {
            motion.relative_orientation()
        } else {
            let bodies = input.model().bodies();

            let i_body = bodies
                .get(joint.i_marker().body_id())
                .expect("validated model contains i body");

            let j_body = bodies
                .get(joint.j_marker().body_id())
                .expect("validated model contains j body");

            let i_position = i_body.position()
                + i_body
                    .orientation()
                    .transform_vector(&joint.i_marker().position());
            let j_position = j_body.position()
                + j_body
                    .orientation()
                    .transform_vector(&joint.j_marker().position());
            let distance = (i_position - j_position).norm();

            if distance > REFERENCE_POSITION_TOLERANCE {
                return Err(JointCoordinateError::InconsistentReferenceMarkers {
                    joint_id: joint.id(),
                    distance,
                    tolerance: REFERENCE_POSITION_TOLERANCE,
                });
            }

            // Calculate the relative orientation of the joint based on the
            // orientations of the bodies and joint marker orientations
            let i_orientation = i_body.orientation() * joint.i_marker().orientation();
            let j_orientation = j_body.orientation() * joint.j_marker().orientation();

            i_orientation.inverse() * j_orientation
        };

        values.insert(joint.id(), JointCoordinate::new(relative_orientation));
    }

    Ok(JointCoordinates { values })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum JointCoordinateError {
    #[error("motion `{motion_name}` references unknown joint `{joint_id:?}`")]
    UnknownJoint {
        motion_name: String,
        joint_id: JointId,
    },
    #[error("joint `{joint_id:?}` has two motions: `{first_motion}` and `{second_motion}`")]
    DuplicateMotion {
        joint_id: JointId,
        first_motion: String,
        second_motion: String,
    },
    #[error(
        "joint `{joint_id:?}` reference markers differ by {distance:e}, exceeding tolerance {tolerance:e}"
    )]
    InconsistentReferenceMarkers {
        joint_id: JointId,
        distance: f64,
        tolerance: f64,
    },
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::model::{
        Bodies, Body, BodyId, Joint, JointKind, Joints, Marker, Model, Motion, MotionKind,
    };

    #[test]
    fn derives_missing_reference_coordinate() {
        let input = input(vec![], Vector3::new(0.0, 0.0, 100.0));

        let coordinates = resolve_joint_coordinates(&input).unwrap();
        let orientation = coordinates
            .get(JointId::new(1))
            .unwrap()
            .relative_orientation();

        assert!(orientation.angle() < 1.0e-12);
    }

    #[test]
    fn uses_requested_coordinate() {
        let requested =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
        let input = input(
            vec![motion("rotate", JointId::new(1), requested)],
            Vector3::new(0.0, 0.0, 100.0),
        );

        let coordinates = resolve_joint_coordinates(&input).unwrap();
        let resolved = coordinates
            .get(JointId::new(1))
            .unwrap()
            .relative_orientation();

        assert!(resolved.angle_to(&requested) < 1.0e-12);
    }

    #[test]
    fn resolves_simultaneous_motion_for_two_joints() {
        let yaml = include_str!("../../tests/fixtures/spherical_two_body_motion.yaml");
        let input = crate::io::yaml::parse_yaml_str(yaml)
            .unwrap()
            .into_input()
            .unwrap();
        let coordinates = resolve_joint_coordinates(&input).unwrap();

        let joint_1 = coordinates
            .get(JointId::new(1))
            .unwrap()
            .relative_orientation();
        let joint_2 = coordinates
            .get(JointId::new(2))
            .unwrap()
            .relative_orientation();
        let expected_joint_1 =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
        let expected_joint_2 =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);

        assert!(joint_1.angle_to(&expected_joint_1) < 1.0e-12);
        assert!(joint_2.angle_to(&expected_joint_2) < 1.0e-12);
    }

    #[test]
    fn rejects_unknown_joint() {
        let input = input(
            vec![motion(
                "unknown",
                JointId::new(99),
                UnitQuaternion::identity(),
            )],
            Vector3::new(0.0, 0.0, 100.0),
        );

        let error = resolve_joint_coordinates(&input).unwrap_err();

        assert!(matches!(
            error,
            JointCoordinateError::UnknownJoint {
                motion_name,
                joint_id,
            } if motion_name == "unknown" && joint_id == JointId::new(99)
        ));
    }

    #[test]
    fn rejects_duplicate_motion() {
        let input = input(
            vec![
                motion("first", JointId::new(1), UnitQuaternion::identity()),
                motion("second", JointId::new(1), UnitQuaternion::identity()),
            ],
            Vector3::new(0.0, 0.0, 100.0),
        );

        let error = resolve_joint_coordinates(&input).unwrap_err();

        assert!(matches!(
            error,
            JointCoordinateError::DuplicateMotion {
                joint_id,
                first_motion,
                second_motion,
            } if joint_id == JointId::new(1)
                && first_motion == "first"
                && second_motion == "second"
        ));
    }

    #[test]
    fn rejects_inconsistent_reference_markers() {
        let input = input(vec![], Vector3::zeros());

        let error = resolve_joint_coordinates(&input).unwrap_err();

        assert!(matches!(
            error,
            JointCoordinateError::InconsistentReferenceMarkers {
                joint_id,
                distance,
                tolerance,
            } if joint_id == JointId::new(1)
                && (distance - 100.0).abs() < 1.0e-12
                && tolerance == REFERENCE_POSITION_TOLERANCE
        ));
    }

    fn input(motions: Vec<Motion>, child_marker_position: Vector3<f64>) -> Input {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, Vector3::zeros()),
            body(BodyId::new(1), Vector3::new(0.0, 0.0, -100.0)),
        ])
        .unwrap();
        let joints = Joints::new(vec![Joint::new(
            JointId::new(1),
            "joint",
            JointKind::Spherical,
            marker(BodyId::GROUND, Vector3::zeros()),
            marker(BodyId::new(1), child_marker_position),
        )])
        .unwrap();

        Input::new(Model::new(bodies, joints).unwrap(), motions)
    }

    fn body(id: BodyId, position: Vector3<f64>) -> Body {
        Body::new(
            id,
            format!("body {id:?}"),
            position,
            UnitQuaternion::identity(),
            vec![],
        )
    }

    fn marker(body_id: BodyId, position: Vector3<f64>) -> Marker {
        Marker::new("marker", body_id, position, UnitQuaternion::identity())
    }

    fn motion(name: &str, joint_id: JointId, relative_orientation: UnitQuaternion<f64>) -> Motion {
        Motion::new(
            name,
            MotionKind::JointCoordinates,
            joint_id,
            relative_orientation,
        )
    }
}
