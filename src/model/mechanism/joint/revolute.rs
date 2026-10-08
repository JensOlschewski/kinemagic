use nalgebra::{UnitQuaternion, Vector3};

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

    pub fn component_count(&self) -> usize {
        1
    }

    pub fn free_component_indices(&self) -> Vec<usize> {
        self.angle.is_none().then_some(0).into_iter().collect()
    }

    /// Returns the relative orientation resulting from the prescribed
    /// angular displacement.
    ///
    /// An unprescribed angle is assumed to be zero.
    pub fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.relative_orientation_for(0.0)
    }

    pub fn relative_orientation_for(&self, candidate: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.resolve_angle(candidate))
            * self.reference_orientation
    }

    pub fn relative_orientation_at(&self, time: f64, candidate: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.resolve_angle_at(time, candidate))
            * self.reference_orientation
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

    /// Relative angular velocity for a given rate of change of the hinge
    /// angle, expressed in the parent marker frame.
    ///
    /// The hinge axis is fixed at the marker-local Z axis, so unlike a
    /// spherical joint this needs no Rodrigues-formula mapping: the angular
    /// velocity is simply the rate about that axis.
    pub fn relative_angular_velocity(&self, angle_rate: f64) -> Vector3<f64> {
        Vector3::z() * angle_rate
    }

    pub fn reverse_relative_angular_velocity(&self, angle: f64, angle_rate: f64) -> Vector3<f64> {
        let forward = self.relative_angular_velocity(angle_rate);
        let orientation = self.relative_orientation_for(angle);

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
            coordinate.relative_angular_velocity(0.7),
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
        let forward = coordinate.relative_angular_velocity(0.6);
        let reverse = coordinate.reverse_relative_angular_velocity(0.0, 0.6);

        assert!((reverse + forward).norm() < 1.0e-12);
    }
}
