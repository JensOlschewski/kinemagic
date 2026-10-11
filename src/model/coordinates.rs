use crate::model::mechanism::joint::revolute::RevoluteCoordinate;
use crate::model::mechanism::joint::spherical::SphericalCoordinate;
use crate::model::mechanism::{JointId, JointKind, Mechanism};
use crate::model::motion::Motions;

use nalgebra::{UnitQuaternion, Vector3};
use std::collections::BTreeMap;
use std::ops::Range;
use std::sync::atomic::{AtomicUsize, Ordering};
use thiserror::Error;

const REFERENCE_POSITION_TOLERANCE: f64 = 1.0e-9;
static NEXT_LAYOUT_ID: AtomicUsize = AtomicUsize::new(1);

/// Behavior every joint's coordinate type must provide: how its
/// generalized coordinate(s) map to a relative orientation and angular
/// velocity. One joint kind, one impl.
///
/// Pose propagation and the position part of closure residuals are the same
/// geometric fact for every joint kind (markers coincide) and call the
/// shared `joint::geometry` functions directly, without going through this
/// trait.
pub trait JointCoordinate {
    fn reference_orientation(&self) -> UnitQuaternion<f64>;
    fn component_count(&self) -> usize;
    fn free_component_indices(&self) -> Vec<usize>;
    fn relative_orientation_at(&self, time: f64, candidate: Vector3<f64>) -> UnitQuaternion<f64>;
    fn resolve_displacement_at(&self, time: f64, candidate: Vector3<f64>) -> Vector3<f64>;
    fn relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64>;
    fn reverse_relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64>;

    /// Orientation closure residual targets at `time`: which of the three
    /// world-frame rotation-vector axes (x=0, y=1, z=2) this joint
    /// constrains for a closed loop, and what value each one must equal.
    ///
    /// The caller computes `actual_displacement` (the scaled-axis deviation
    /// of the actual relative marker orientation from this coordinate's
    /// `reference_orientation`) and reads off a residual
    /// `actual_displacement[axis] - target` for each returned pair.
    fn closure_orientation_residual_targets(&self, time: f64) -> Vec<(usize, f64)>;

    fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.relative_orientation_for(Vector3::zeros())
    }

    fn relative_orientation_for(&self, candidate: Vector3<f64>) -> UnitQuaternion<f64> {
        self.relative_orientation_at(0.0, candidate)
    }
}

/// One of the known joint coordinate kinds, picked at runtime.
#[derive(Debug, Clone, Copy)]
pub enum AnyJointCoordinate {
    Spherical(SphericalCoordinate),
    Revolute(RevoluteCoordinate),
}

impl AnyJointCoordinate {
    /// The only place that matches on the concrete variant.
    fn as_trait(&self) -> &dyn JointCoordinate {
        match self {
            AnyJointCoordinate::Spherical(coordinate) => coordinate,
            AnyJointCoordinate::Revolute(coordinate) => coordinate,
        }
    }
}

impl JointCoordinate for AnyJointCoordinate {
    fn reference_orientation(&self) -> UnitQuaternion<f64> {
        self.as_trait().reference_orientation()
    }

    fn component_count(&self) -> usize {
        self.as_trait().component_count()
    }

    fn free_component_indices(&self) -> Vec<usize> {
        self.as_trait().free_component_indices()
    }

    fn relative_orientation_at(&self, time: f64, candidate: Vector3<f64>) -> UnitQuaternion<f64> {
        self.as_trait().relative_orientation_at(time, candidate)
    }

    fn resolve_displacement_at(&self, time: f64, candidate: Vector3<f64>) -> Vector3<f64> {
        self.as_trait().resolve_displacement_at(time, candidate)
    }

    fn relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        self.as_trait()
            .relative_angular_velocity(displacement, displacement_rate)
    }

    fn reverse_relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        self.as_trait()
            .reverse_relative_angular_velocity(displacement, displacement_rate)
    }

    fn closure_orientation_residual_targets(&self, time: f64) -> Vec<(usize, f64)> {
        self.as_trait().closure_orientation_residual_targets(time)
    }
}

#[derive(Debug)]
pub struct CoordinateLayout {
    id: usize,
    primary_coordinates: Vec<(JointId, usize)>,
    free_primary_coordinates: Vec<(JointId, usize)>,
    primary_columns: BTreeMap<JointId, Vec<usize>>,
}

impl CoordinateLayout {
    pub fn new(
        primary_coordinates: Vec<(JointId, usize)>,
        free_primary_coordinates: Vec<(JointId, usize)>,
    ) -> Self {
        let mut primary_columns = BTreeMap::new();
        for (column, (joint_id, _)) in primary_coordinates.iter().enumerate() {
            primary_columns
                .entry(*joint_id)
                .or_insert_with(Vec::new)
                .push(column);
        }
        Self {
            id: NEXT_LAYOUT_ID.fetch_add(1, Ordering::Relaxed),
            primary_coordinates,
            free_primary_coordinates,
            primary_columns,
        }
    }

    pub fn primary_coordinates(&self) -> &[(JointId, usize)] {
        &self.primary_coordinates
    }

    pub fn free_primary_coordinates(&self) -> &[(JointId, usize)] {
        &self.free_primary_coordinates
    }

    pub fn primary_coordinate_count(&self) -> usize {
        self.primary_coordinates.len()
    }

    pub fn primary_range(&self, joint_id: JointId) -> Option<Range<usize>> {
        let columns = self.primary_columns.get(&joint_id)?;
        let start = *columns.first()?;
        let end = columns.last()? + 1;
        (columns.len() == end - start).then_some(start..end)
    }

    pub(crate) fn id(&self) -> usize {
        self.id
    }
}

#[derive(Debug)]
pub struct JointCoordinates {
    values: BTreeMap<JointId, AnyJointCoordinate>,
}

impl JointCoordinates {
    pub fn get(&self, joint_id: JointId) -> Option<&AnyJointCoordinate> {
        self.values.get(&joint_id)
    }
}

pub fn resolve_joint_coordinates(
    mechanism: &Mechanism,
    motions: &Motions,
) -> Result<JointCoordinates, JointCoordinateError> {
    let mut motions_by_joint = BTreeMap::new();
    let joints = mechanism.joints();

    for motion in motions {
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
        let bodies = mechanism.bodies();

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
        let motion = motions_by_joint.get(&joint.id()).copied();

        let coordinate = match joint.kind() {
            JointKind::Spherical => AnyJointCoordinate::Spherical(
                SphericalCoordinate::from_reference(reference_orientation, motion),
            ),
            JointKind::Revolute => AnyJointCoordinate::Revolute(
                RevoluteCoordinate::from_reference(joint.id(), reference_orientation, motion)?,
            ),
        };

        values.insert(joint.id(), coordinate);
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
    #[error(
        "revolute joint `{joint_id:?}` reference marker hinge axes differ by {misalignment:e}, exceeding tolerance {tolerance:e}"
    )]
    MisalignedHingeAxes {
        joint_id: JointId,
        misalignment: f64,
        tolerance: f64,
    },
    #[error("motion `{motion_name}` prescribes rot_x/rot_y on revolute joint `{joint_id:?}`")]
    IncompatibleMotion {
        joint_id: JointId,
        motion_name: String,
    },
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::model::mechanism::*;
    use crate::model::motion::*;

    #[test]
    fn derives_missing_reference_coordinate() {
        let input = input(vec![], Vector3::new(0.0, 0.0, 100.0));

        let coordinates = resolve(&input).unwrap();
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

        let coordinates = resolve(&input).unwrap();
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

        let error = resolve(&input).unwrap_err();

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

        let error = resolve(&input).unwrap_err();

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

        let error = resolve(&input).unwrap_err();

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
    fn resolves_revolute_coordinate_with_free_angle() {
        let input = revolute_input(vec![], UnitQuaternion::identity());

        let coordinates = resolve(&input).unwrap();
        let coordinate = coordinates.get(JointId::new(1)).unwrap();

        assert_eq!(coordinate.component_count(), 1);
        assert_eq!(coordinate.free_component_indices(), vec![0]);
        assert!(coordinate.relative_orientation().angle() < 1.0e-12);
    }

    #[test]
    fn resolves_revolute_coordinate_with_prescribed_angle() {
        let motion = Motion::new(
            "rotate",
            MotionKind::JointCoordinates,
            JointId::new(1),
            JointDisplacement::new(Vector3::new(None, None, Some(std::f64::consts::FRAC_PI_2))),
        );
        let input = revolute_input(vec![motion], UnitQuaternion::identity());

        let coordinates = resolve(&input).unwrap();
        let coordinate = coordinates.get(JointId::new(1)).unwrap();
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);

        assert_eq!(coordinate.free_component_indices(), Vec::<usize>::new());
        assert!(coordinate.relative_orientation().angle_to(&expected) < 1.0e-12);
    }

    #[test]
    fn rejects_revolute_motion_with_x_or_y_component() {
        let motion = Motion::new(
            "tilt",
            MotionKind::JointCoordinates,
            JointId::new(1),
            JointDisplacement::new(Vector3::new(Some(0.1), None, None)),
        );
        let input = revolute_input(vec![motion], UnitQuaternion::identity());

        let error = resolve(&input).unwrap_err();

        assert!(matches!(
            error,
            JointCoordinateError::IncompatibleMotion { joint_id, motion_name }
                if joint_id == JointId::new(1) && motion_name == "tilt"
        ));
    }

    #[test]
    fn rejects_misaligned_revolute_hinge_axes() {
        let tilted =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
        let input = revolute_input(vec![], tilted);

        let error = resolve(&input).unwrap_err();

        assert!(matches!(
            error,
            JointCoordinateError::MisalignedHingeAxes { joint_id, .. }
                if joint_id == JointId::new(1)
        ));
    }

    /// Builds a one-body revolute mechanism with aligned reference markers at
    /// the same world position, with `j_marker_orientation` applied on top of
    /// the J marker so tests can introduce a hinge-axis misalignment.
    fn revolute_input(
        motions: Vec<Motion>,
        j_marker_orientation: UnitQuaternion<f64>,
    ) -> (Mechanism, Motions) {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, Vector3::zeros()),
            body(BodyId::new(1), Vector3::zeros()),
        ])
        .unwrap();

        let joints = Joints::new(vec![Joint::new(
            JointId::new(1),
            "joint",
            JointKind::Revolute,
            JointRole::Auto,
            marker(BodyId::GROUND, Vector3::zeros()),
            Marker::new("j", BodyId::new(1), Vector3::zeros(), j_marker_orientation),
        )])
        .unwrap();

        (
            Mechanism::new(bodies, joints).unwrap(),
            Motions::new(motions),
        )
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

        let input = (
            Mechanism::new(bodies, joints).unwrap(),
            Motions::new(vec![motion("rotate", JointId::new(1), requested)]),
        );

        let coordinates = resolve(&input).unwrap();
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
    fn layout_preserves_deterministic_contiguous_joint_columns() {
        let layout = CoordinateLayout::new(
            vec![
                (JointId::new(2), 0),
                (JointId::new(2), 1),
                (JointId::new(2), 2),
                (JointId::new(9), 0),
                (JointId::new(9), 1),
                (JointId::new(9), 2),
            ],
            vec![(JointId::new(2), 1), (JointId::new(9), 0)],
        );

        assert_eq!(layout.primary_coordinates()[0], (JointId::new(2), 0));
        assert_eq!(layout.primary_range(JointId::new(2)), Some(0..3));
        assert_eq!(layout.primary_range(JointId::new(9)), Some(3..6));
        assert_eq!(layout.primary_coordinate_count(), 6);
    }

    fn resolve(input: &(Mechanism, Motions)) -> Result<JointCoordinates, JointCoordinateError> {
        resolve_joint_coordinates(&input.0, &input.1)
    }

    fn input(motions: Vec<Motion>, child_marker_position: Vector3<f64>) -> (Mechanism, Motions) {
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

        (
            Mechanism::new(bodies, joints).unwrap(),
            Motions::new(motions),
        )
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
