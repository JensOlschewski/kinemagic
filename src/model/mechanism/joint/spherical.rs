use nalgebra::{Matrix3, SMatrix, UnitQuaternion, Vector3};

use crate::model::mechanism::Marker;
use crate::model::motion::JointDisplacement;
use crate::model::spatial::BodyPose;

pub fn spherical_child_pose(
    parent: &BodyPose,
    i_marker: &Marker,
    j_marker: &Marker,
    relative_orientation: UnitQuaternion<f64>,
) -> BodyPose {
    // R_child = R_parent A_i Q A_j^-1
    let child_orientation = parent.marker_orientation(i_marker)
        * relative_orientation
        * j_marker.orientation().inverse();

    // r_child = p_i_world - R_child s_j
    let child_position =
        parent.marker_position(i_marker) - child_orientation.transform_vector(&j_marker.position());
    BodyPose::new(child_position, child_orientation)
}

/// Jacobian blocks of the spherical position residual
/// `p_i(marker) - p_j(marker)` with respect to the world-frame twists
/// `[v; ω]` of the I and J bodies.
///
/// The fixed body-local marker offsets are mapped to world with each body's
/// current rotation: `skew(R s) = R skew(s) Rᵀ`.
pub fn position_constraint_blocks(
    i_pose: &BodyPose,
    i_marker: &Marker,
    j_pose: &BodyPose,
    j_marker: &Marker,
) -> (SMatrix<f64, 3, 6>, SMatrix<f64, 3, 6>) {
    let i_rotation = i_pose.orientation().to_rotation_matrix().into_inner();
    let j_rotation = j_pose.orientation().to_rotation_matrix().into_inner();
    let i_skew = i_rotation * i_marker.position().cross_matrix() * i_rotation.transpose();
    let j_skew = j_rotation * j_marker.position().cross_matrix() * j_rotation.transpose();

    let mut gi = SMatrix::<f64, 3, 6>::zeros();
    gi.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&Matrix3::identity());
    gi.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-i_skew));

    let mut gj = SMatrix::<f64, 3, 6>::zeros();
    gj.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(-Matrix3::identity()));
    gj.fixed_view_mut::<3, 3>(0, 3).copy_from(&j_skew);

    (gi, gj)
}

/// Spherical joint coordinate: reference orientation plus prescribed or free
/// rotation-vector components.
#[derive(Debug, Clone, Copy)]
pub struct SphericalCoordinate {
    reference_orientation: UnitQuaternion<f64>,
    displacement: JointDisplacement,
}

impl SphericalCoordinate {
    pub fn new(
        reference_orientation: UnitQuaternion<f64>,
        displacement: JointDisplacement,
    ) -> Self {
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

    pub fn component_count(&self) -> usize {
        3
    }

    pub fn free_component_indices(&self) -> Vec<usize> {
        let rotation = self.displacement.rotation();

        [rotation.x, rotation.y, rotation.z]
            .into_iter()
            .enumerate()
            .filter_map(|(index, value)| value.is_none().then_some(index))
            .collect()
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

    pub fn relative_orientation_at(
        &self,
        time: f64,
        candidate: Vector3<f64>,
    ) -> UnitQuaternion<f64> {
        UnitQuaternion::from_scaled_axis(self.resolve_displacement_at(time, candidate))
            * self.reference_orientation
    }

    /// Resolves the joint displacement.
    ///
    /// Components prescribed by the joint override the corresponding components
    /// of `candidate`. Unprescribed components are taken from `candidate`.
    pub fn resolve_displacement(&self, candidate: Vector3<f64>) -> Vector3<f64> {
        self.resolve_displacement_at(0.0, candidate)
    }

    pub fn resolve_displacement_at(&self, time: f64, candidate: Vector3<f64>) -> Vector3<f64> {
        let displacement = self.displacement.at(time);
        let prescribed = displacement.rotation();

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

    pub fn displacement_rate_from_relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        angular_velocity: Vector3<f64>,
    ) -> Vector3<f64> {
        let angle = displacement.norm();
        let skew = displacement.cross_matrix();
        let skew_squared = skew * skew;
        let second = if angle < 1.0e-8 {
            1.0 / 12.0 + angle.powi(2) / 720.0
        } else {
            1.0 / angle.powi(2) - 1.0 / (2.0 * angle) * (angle / 2.0).cos() / (angle / 2.0).sin()
        };

        (Matrix3::identity() - 0.5 * skew + second * skew_squared) * angular_velocity
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::model::mechanism::BodyId;
    use crate::model::spatial::BodyTwist;

    #[test]
    fn preserves_marker_coincidence_with_offsets() {
        let parent = BodyPose::new(Vector3::new(1.0, 2.0, 3.0), UnitQuaternion::identity());
        let i_marker = Marker::new(
            "i",
            BodyId::GROUND,
            Vector3::new(2.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let j_marker = Marker::new(
            "j",
            BodyId::new(1),
            Vector3::new(0.0, 1.0, 0.0),
            UnitQuaternion::identity(),
        );
        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        assert!(
            (parent.marker_position(&i_marker) - child.marker_position(&j_marker)).norm() < 1.0e-12
        );
        assert!(child.orientation().angle_to(&relative_orientation) < 1.0e-12);
    }

    #[test]
    fn position_constraint_blocks_map_body_twists_to_marker_velocity_difference() {
        let i_pose = BodyPose::new(
            Vector3::new(1.0, -2.0, 0.5),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.3, -0.4, 0.2)),
        );
        let j_pose = BodyPose::new(
            Vector3::new(-0.5, 1.0, 2.0),
            UnitQuaternion::from_scaled_axis(Vector3::new(-0.1, 0.5, 0.7)),
        );
        let i_marker = Marker::new(
            "i",
            BodyId::new(1),
            Vector3::new(0.3, 0.1, -0.2),
            UnitQuaternion::identity(),
        );
        let j_marker = Marker::new(
            "j",
            BodyId::new(2),
            Vector3::new(-0.4, 0.2, 0.6),
            UnitQuaternion::identity(),
        );
        let i_twist = BodyTwist::new(Vector3::new(0.2, 0.1, -0.3), Vector3::new(0.5, -0.2, 0.4));
        let j_twist = BodyTwist::new(Vector3::new(-0.1, 0.6, 0.2), Vector3::new(0.3, 0.7, -0.5));

        let (gi, gj) = position_constraint_blocks(&i_pose, &i_marker, &j_pose, &j_marker);
        let stack = |twist: &BodyTwist| {
            nalgebra::SVector::<f64, 6>::from_column_slice(&[
                twist.linear_velocity().x,
                twist.linear_velocity().y,
                twist.linear_velocity().z,
                twist.angular_velocity().x,
                twist.angular_velocity().y,
                twist.angular_velocity().z,
            ])
        };
        let expected =
            i_pose.marker_velocity(&i_marker, i_twist) - j_pose.marker_velocity(&j_marker, j_twist);

        assert!((gi * stack(&i_twist) + gj * stack(&j_twist) - expected).norm() < 1.0e-12);
    }

    #[test]
    fn maps_scaled_axis_rate_to_angular_velocity() {
        let coordinate = SphericalCoordinate::new(
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
        let coordinate = SphericalCoordinate::new(
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
        let coordinate = SphericalCoordinate::new(
            reference_orientation,
            JointDisplacement::new(Vector3::new(Some(0.4), None, Some(-0.6))),
        );
        let candidate = Vector3::new(1.0, 0.5, 2.0);
        let expected =
            UnitQuaternion::from_scaled_axis(Vector3::new(0.4, 0.5, -0.6)) * reference_orientation;

        assert!(
            coordinate
                .relative_orientation_for(candidate)
                .angle_to(&expected)
                < 1.0e-12
        );
    }

    #[test]
    fn evaluates_linear_displacement_at_requested_time() {
        let coordinate = SphericalCoordinate::new(
            UnitQuaternion::identity(),
            JointDisplacement::with_rates(
                Vector3::new(Some(0.2), None, Some(-0.6)),
                Vector3::new(Some(0.1), None, Some(0.3)),
            ),
        );

        let expected = Vector3::new(0.4, 0.5, -0.0);
        assert!(
            (coordinate.resolve_displacement_at(2.0, Vector3::new(9.0, 0.5, 9.0)) - expected)
                .norm()
                < 1.0e-12
        );
    }

    #[test]
    fn lists_free_components_in_authored_order() {
        let coordinate = SphericalCoordinate::new(
            UnitQuaternion::identity(),
            JointDisplacement::new(Vector3::new(Some(0.4), None, Some(-0.6))),
        );

        assert_eq!(coordinate.free_component_indices(), vec![1]);
    }
}
