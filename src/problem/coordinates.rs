use crate::model::{Input, JointDisplacement, JointId};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
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
}

#[derive(Debug, Clone, Copy)]
pub struct JointCoordinate {
    reference_orientation: UnitQuaternion<f64>,
    displacement: JointDisplacement,
}

impl JointCoordinate {
    fn new(reference_orientation: UnitQuaternion<f64>, displacement: JointDisplacement) -> Self {
        Self {
            reference_orientation,
            displacement,
        }
    }

    pub fn reference_orientation(&self) -> UnitQuaternion<f64> {
        self.reference_orientation
    }

    pub fn displacement(&self) -> &JointDisplacement {
        &self.displacement
    }

    /// Returns the relative orientation resulting from the prescribed
    /// rotational displacement.
    ///
    /// Unprescribed rotation components are assumed to be zero.
    pub fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.relative_orientation_for(Vector3::zeros())
    }

    pub fn relative_orientation_for(&self, candidate: Vector3<f64>) -> UnitQuaternion<f64> {
        UnitQuaternion::from_scaled_axis(self.resolve_displacement(candidate))
            * self.reference_orientation
    }

    /// Resolves the joint displacement.
    ///
    /// Components prescribed by the joint override the corresponding components
    /// of `candidate`. Unprescribed components are taken from `candidate`.
    pub fn resolve_displacement(&self, candidate: Vector3<f64>) -> Vector3<f64> {
        let prescribed = self.displacement.rotation();

        Vector3::new(
            prescribed.x.unwrap_or(candidate.x),
            prescribed.y.unwrap_or(candidate.y),
            prescribed.z.unwrap_or(candidate.z),
        )
    }

    pub fn relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        let angle = displacement.norm();
        let skew = displacement.cross_matrix();
        let skew_squared = skew * skew;

        let (first, second) = if angle < 1.0e-8 {
            (
                0.5 - angle.powi(2) / 24.0,
                1.0 / 6.0 - angle.powi(2) / 120.0,
            )
        } else {
            (
                (1.0 - angle.cos()) / angle.powi(2),
                (angle - angle.sin()) / angle.powi(3),
            )
        };

        (Matrix3::identity() + first * skew + second * skew_squared) * displacement_rate
    }

    pub fn reverse_relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        let forward = self.relative_angular_velocity(displacement, displacement_rate);
        let orientation = self.relative_orientation_for(displacement);

        -orientation.inverse_transform_vector(&forward)
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

        let reference_orientation = i_orientation.inverse() * j_orientation;

        let displacement = if let Some(motion) = motions_by_joint.get(&joint.id()) {
            *motion.joint_displacement()
        } else {
            JointDisplacement::new(nalgebra::Vector3::new(None, None, None))
        };

        values.insert(
            joint.id(),
            JointCoordinate::new(reference_orientation, displacement),
        );
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
    use crate::model::*;

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
        let requested = Vector3::new(Some(std::f64::consts::FRAC_PI_2), None, None);
        let input = input(
            vec![motion("rotate", JointId::new(1), requested)],
            Vector3::new(0.0, 0.0, 100.0),
        );

        let delta = requested.map(|component| component.unwrap_or(0.0));
        let expected_orientation = UnitQuaternion::from_scaled_axis(delta);

        let coordinates = resolve_joint_coordinates(&input).unwrap();
        let resolved = coordinates
            .get(JointId::new(1))
            .unwrap()
            .relative_orientation();

        assert!(resolved.angle_to(&expected_orientation) < 1.0e-12);
    }

    #[test]
    fn rejects_unknown_joint() {
        let requested = Vector3::new(Some(0.0), None, None);

        let input = input(
            vec![motion("unknown", JointId::new(99), requested)],
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
        let requested = Vector3::new(Some(0.0), None, None);
        let input = input(
            vec![
                motion("first", JointId::new(1), requested),
                motion("second", JointId::new(1), requested),
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

    #[test]
    fn applies_displacement_in_reference_i_marker_frame() {
        let reference_orientation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.4);
        let requested = Vector3::new(Some(std::f64::consts::FRAC_PI_2), None, None);

        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, Vector3::zeros()),
            body(BodyId::new(1), Vector3::new(0.0, 0.0, -100.0)),
        ])
        .unwrap();

        let joints = Joints::new(vec![Joint::new(
            JointId::new(1),
            "joint",
            JointKind::Spherical,
            JointRole::Auto,
            marker(BodyId::GROUND, Vector3::zeros()),
            Marker::new(
                "j",
                BodyId::new(1),
                Vector3::new(0.0, 0.0, 100.0),
                reference_orientation,
            ),
        )])
        .unwrap();

        let input = Input::new(
            Model::new(bodies, joints).unwrap(),
            vec![motion("rotate", JointId::new(1), requested)],
        );

        let coordinates = resolve_joint_coordinates(&input).unwrap();
        let actual = coordinates
            .get(JointId::new(1))
            .unwrap()
            .relative_orientation();

        let delta_orientation =
            UnitQuaternion::from_scaled_axis(Vector3::new(std::f64::consts::FRAC_PI_2, 0.0, 0.0));
        let expected = delta_orientation * reference_orientation;

        assert!(actual.angle_to(&expected) < 1.0e-12);
    }

    #[test]
    fn maps_scaled_axis_rate_to_angular_velocity() {
        let coordinate = JointCoordinate::new(
            UnitQuaternion::identity(),
            JointDisplacement::new(Vector3::new(None, None, None)),
        );
        let displacement = Vector3::new(0.4, -0.3, 0.2);
        let displacement_rate = Vector3::new(-0.2, 0.5, 0.7);
        let angular_velocity =
            coordinate.relative_angular_velocity(displacement, displacement_rate);
        let step = 1.0e-7;
        let forward = UnitQuaternion::from_scaled_axis(displacement + step * displacement_rate);
        let backward = UnitQuaternion::from_scaled_axis(displacement - step * displacement_rate);
        let finite_difference = (forward * backward.inverse()).scaled_axis() / (2.0 * step);

        assert!((angular_velocity - finite_difference).norm() < 1.0e-9);
    }

    #[test]
    fn maps_small_scaled_axis_rate_without_singularity() {
        let coordinate = JointCoordinate::new(
            UnitQuaternion::identity(),
            JointDisplacement::new(Vector3::new(None, None, None)),
        );
        let displacement = Vector3::new(1.0e-10, -2.0e-10, 3.0e-10);
        let displacement_rate = Vector3::new(0.4, -0.5, 0.6);

        assert!(
            (coordinate.relative_angular_velocity(displacement, displacement_rate)
                - displacement_rate)
                .norm()
                < 1.0e-9
        );
    }

    #[test]
    fn resolves_prescribed_and_candidate_orientation_components() {
        let reference_orientation = UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3));
        let coordinate = JointCoordinate::new(
            reference_orientation,
            JointDisplacement::new(Vector3::new(Some(0.4), None, Some(-0.6))),
        );
        let candidate = Vector3::new(1.0, 0.5, 2.0);
        let expected = UnitQuaternion::from_scaled_axis(Vector3::new(0.4, 0.5, -0.6))
            * reference_orientation;

        assert!(
            coordinate
                .relative_orientation_for(candidate)
                .angle_to(&expected)
                < 1.0e-12
        );
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
            JointRole::Auto,
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

    fn motion(name: &str, joint_id: JointId, rotation: Vector3<Option<f64>>) -> Motion {
        Motion::new(
            name,
            MotionKind::JointCoordinates,
            joint_id,
            JointDisplacement::new(rotation),
        )
    }
}
