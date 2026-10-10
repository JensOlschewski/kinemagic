use nalgebra::{UnitQuaternion, Vector3};

use crate::model::coordinates::{JointCoordinate, JointCoordinateError};
use crate::model::mechanism::JointId;
use crate::model::motion::Motion;

const REFERENCE_AXIS_TOLERANCE: f64 = 1.0e-9;

/// Revolute joint coordinate: reference orientation plus a single prescribed
/// or free rotation about the marker-local Z axis (the hinge axis).
#[derive(Debug, Clone, Copy)]
pub struct RevoluteCoordinate {
    reference_orientation: UnitQuaternion<f64>,
    angle: Option<f64>,
    angle_rate: f64,
}

impl RevoluteCoordinate {
    pub fn new(
        reference_orientation: UnitQuaternion<f64>,
        angle: Option<f64>,
        angle_rate: f64,
    ) -> Self {
        Self {
            reference_orientation,
            angle,
            angle_rate,
        }
    }

    pub fn reference_orientation(&self) -> UnitQuaternion<f64> {
        self.reference_orientation
    }

    /// Builds this joint's coordinate from its reference orientation and
    /// prescribed motion, if any.
    ///
    /// A single hinge angle can only reproduce the reference pose when both
    /// markers' local Z axes already coincide in world space — equivalently,
    /// when `reference_orientation` leaves the Z axis invariant. Otherwise
    /// the "missing" tilt has nowhere to go.
    pub fn from_reference(
        joint_id: JointId,
        reference_orientation: UnitQuaternion<f64>,
        motion: Option<&Motion>,
    ) -> Result<Self, JointCoordinateError> {
        let axis = Vector3::z();
        let misalignment = (axis - reference_orientation.transform_vector(&axis)).norm();

        if misalignment > REFERENCE_AXIS_TOLERANCE {
            return Err(JointCoordinateError::MisalignedHingeAxes {
                joint_id,
                misalignment,
                tolerance: REFERENCE_AXIS_TOLERANCE,
            });
        }

        let (angle, angle_rate) = match motion {
            Some(motion) => {
                let rotation = *motion.joint_displacement().rotation();
                if rotation.x.is_some() || rotation.y.is_some() {
                    return Err(JointCoordinateError::IncompatibleMotion {
                        joint_id,
                        motion_name: motion.name().to_owned(),
                    });
                }

                let rate = motion.joint_displacement().rotation_rate().z.unwrap_or(0.0);
                (rotation.z, rate)
            }
            None => (None, 0.0),
        };

        Ok(Self::new(reference_orientation, angle, angle_rate))
    }

    /// Resolves the joint angle.
    ///
    /// A prescribed angle overrides `candidate`. An unprescribed angle takes
    /// `candidate` as-is.
    pub fn resolve_angle(&self, candidate: f64) -> f64 {
        self.resolve_angle_at(0.0, candidate)
    }

    pub fn resolve_angle_at(&self, time: f64, candidate: f64) -> f64 {
        self.angle
            .map(|angle| angle + self.angle_rate * time)
            .unwrap_or(candidate)
    }
}

impl JointCoordinate for RevoluteCoordinate {
    fn component_count(&self) -> usize {
        1
    }

    fn free_component_indices(&self) -> Vec<usize> {
        self.angle.is_none().then_some(0).into_iter().collect()
    }

    fn relative_orientation_at(&self, time: f64, candidate: Vector3<f64>) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            self.resolve_angle_at(time, candidate.x),
        )
    }

    fn resolve_displacement_at(&self, time: f64, candidate: Vector3<f64>) -> Vector3<f64> {
        Vector3::new(self.resolve_angle_at(time, candidate.x), 0.0, 0.0)
    }

    /// Relative angular velocity for a given rate of change of the hinge
    /// angle, expressed in the parent marker frame.
    ///
    /// The hinge axis is fixed at the marker-local Z axis, so unlike a
    /// spherical joint this needs no Rodrigues-formula mapping: the angular
    /// velocity is simply the rate about that axis.
    fn relative_angular_velocity(
        &self,
        _displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        Vector3::z() * displacement_rate.x
    }

    fn reverse_relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        let forward = self.relative_angular_velocity(displacement, displacement_rate);
        let orientation = self.relative_orientation_for(displacement);

        -orientation.inverse_transform_vector(&forward)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_angle_rate_to_angular_velocity_about_hinge_axis() {
        let coordinate = RevoluteCoordinate::new(UnitQuaternion::identity(), None, 0.0);

        assert_eq!(
            coordinate.relative_angular_velocity(Vector3::zeros(), Vector3::new(0.7, 0.0, 0.0)),
            Vector3::new(0.0, 0.0, 0.7)
        );
    }

    #[test]
    fn resolves_prescribed_angle_over_candidate() {
        let coordinate = RevoluteCoordinate::new(UnitQuaternion::identity(), Some(0.4), 0.0);

        assert_eq!(coordinate.resolve_angle(1.0), 0.4);
    }

    #[test]
    fn resolves_candidate_when_angle_is_free() {
        let coordinate = RevoluteCoordinate::new(UnitQuaternion::identity(), None, 0.0);

        assert_eq!(coordinate.resolve_angle(1.0), 1.0);
    }

    #[test]
    fn evaluates_prescribed_angle_at_requested_time() {
        let coordinate = RevoluteCoordinate::new(UnitQuaternion::identity(), Some(0.2), 0.1);

        assert!((coordinate.resolve_angle_at(2.0, 9.0) - 0.4).abs() < 1.0e-12);
    }

    #[test]
    fn lists_free_component_only_when_angle_unprescribed() {
        let free = RevoluteCoordinate::new(UnitQuaternion::identity(), None, 0.0);
        let prescribed = RevoluteCoordinate::new(UnitQuaternion::identity(), Some(0.0), 0.0);

        assert_eq!(free.free_component_indices(), vec![0]);
        assert_eq!(prescribed.free_component_indices(), Vec::<usize>::new());
    }

    #[test]
    fn relative_orientation_rotates_about_reference_marker_z_axis() {
        let coordinate = RevoluteCoordinate::new(UnitQuaternion::identity(), Some(0.5), 0.0);
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5);

        assert!(coordinate.relative_orientation().angle_to(&expected) < 1.0e-12);
    }

    #[test]
    fn reverse_angular_velocity_is_negated_and_frame_transformed() {
        let coordinate = RevoluteCoordinate::new(UnitQuaternion::identity(), None, 0.0);
        let forward =
            coordinate.relative_angular_velocity(Vector3::zeros(), Vector3::new(0.6, 0.0, 0.0));
        let reverse = coordinate
            .reverse_relative_angular_velocity(Vector3::zeros(), Vector3::new(0.6, 0.0, 0.0));

        assert!((reverse + forward).norm() < 1.0e-12);
    }
}
