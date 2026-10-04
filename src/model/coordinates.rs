use crate::model::mechanism::joint::spherical::SphericalCoordinate;
use crate::model::mechanism::{JointId, Mechanism};
use crate::model::motion::{JointDisplacement, Motions};

use nalgebra::{DVector, Vector3};
use std::collections::BTreeMap;
use std::ops::Range;
use std::sync::atomic::{AtomicUsize, Ordering};
use thiserror::Error;

const REFERENCE_POSITION_TOLERANCE: f64 = 1.0e-9;
static NEXT_LAYOUT_ID: AtomicUsize = AtomicUsize::new(1);

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

    fn id(&self) -> usize {
        self.id
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Configuration {
    values: DVector<f64>,
    layout_id: usize,
    primary_coordinates: Vec<(JointId, usize)>,
}

impl Configuration {
    pub fn new(layout: &CoordinateLayout) -> Self {
        Self {
            values: DVector::zeros(layout.primary_coordinate_count()),
            layout_id: layout.id(),
            primary_coordinates: layout.primary_coordinates.clone(),
        }
    }

    pub fn from_values(
        layout: &CoordinateLayout,
        values: DVector<f64>,
    ) -> Result<Self, ConfigurationError> {
        if values.len() == layout.primary_coordinate_count() {
            Ok(Self {
                values,
                layout_id: layout.id(),
                primary_coordinates: layout.primary_coordinates.clone(),
            })
        } else {
            Err(ConfigurationError::DimensionMismatch {
                expected: layout.primary_coordinate_count(),
                actual: values.len(),
            })
        }
    }

    pub fn from_candidates(
        layout: &CoordinateLayout,
        candidates: &BTreeMap<JointId, Vector3<f64>>,
    ) -> Self {
        let mut configuration = Self::new(layout);
        for (joint_id, candidate) in candidates {
            for (column, (_, component)) in layout
                .primary_coordinates
                .iter()
                .enumerate()
                .filter(|(_, (id, _))| id == joint_id)
            {
                configuration.values[column] = candidate[*component];
            }
        }
        configuration
    }

    pub fn values(&self) -> &DVector<f64> {
        &self.values
    }

    pub fn joint_values(&self, joint_id: JointId) -> Vector3<f64> {
        let mut values = Vector3::zeros();
        for (column, (id, component)) in self.primary_coordinates.iter().enumerate() {
            if *id == joint_id {
                values[*component] = self.values[column];
            }
        }
        values
    }

    pub fn validate_layout(&self, layout: &CoordinateLayout) -> Result<(), ConfigurationError> {
        if self.layout_id == layout.id() {
            Ok(())
        } else {
            Err(ConfigurationError::LayoutMismatch)
        }
    }

    pub fn set_primary(&mut self, column: usize, value: f64) -> Result<(), ConfigurationError> {
        let actual = self.values.len();
        let Some(current) = self.values.get_mut(column) else {
            return Err(ConfigurationError::PrimaryColumnOutOfRange { column, actual });
        };
        *current = value;
        Ok(())
    }

    pub fn add_primary(&mut self, column: usize, value: f64) -> Result<(), ConfigurationError> {
        let actual = self.values.len();
        let Some(current) = self.values.get_mut(column) else {
            return Err(ConfigurationError::PrimaryColumnOutOfRange { column, actual });
        };
        *current += value;
        Ok(())
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum ConfigurationError {
    #[error("configuration has {actual} values, expected {expected}")]
    DimensionMismatch { expected: usize, actual: usize },
    #[error("primary coordinate column {column} is out of range for {actual} values")]
    PrimaryColumnOutOfRange { column: usize, actual: usize },
    #[error("configuration belongs to a different coordinate layout")]
    LayoutMismatch,
}

#[derive(Debug)]
pub struct JointCoordinates {
    values: BTreeMap<JointId, SphericalCoordinate>,
}

impl JointCoordinates {
    pub fn get(&self, joint_id: JointId) -> Option<&SphericalCoordinate> {
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

        let displacement = if let Some(motion) = motions_by_joint.get(&joint.id()) {
            *motion.joint_displacement()
        } else {
            JointDisplacement::new(nalgebra::Vector3::new(None, None, None))
        };

        values.insert(
            joint.id(),
            SphericalCoordinate::new(reference_orientation, displacement),
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

    #[test]
    fn configuration_reads_joint_values_and_updates_global_column() {
        let layout = CoordinateLayout::new(
            vec![
                (JointId::new(2), 0),
                (JointId::new(2), 1),
                (JointId::new(2), 2),
            ],
            vec![],
        );
        let mut configuration = Configuration::new(&layout);

        configuration.set_primary(1, 0.5).unwrap();
        configuration.add_primary(1, 0.25).unwrap();

        assert_eq!(configuration.values().len(), 3);
        assert_eq!(
            configuration.joint_values(JointId::new(2)),
            Vector3::new(0.0, 0.75, 0.0)
        );
    }

    #[test]
    fn configuration_uses_exact_interleaved_joint_columns() {
        let layout = CoordinateLayout::new(
            vec![
                (JointId::new(2), 0),
                (JointId::new(9), 1),
                (JointId::new(2), 2),
            ],
            vec![],
        );
        let configuration = Configuration::from_candidates(
            &layout,
            &BTreeMap::from([
                (JointId::new(2), Vector3::new(1.0, 2.0, 3.0)),
                (JointId::new(9), Vector3::new(4.0, 5.0, 6.0)),
            ]),
        );

        assert_eq!(layout.primary_range(JointId::new(2)), None);
        assert_eq!(configuration.values().as_slice(), &[1.0, 5.0, 3.0]);
        assert_eq!(
            configuration.joint_values(JointId::new(2)),
            Vector3::new(1.0, 0.0, 3.0)
        );
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
