use nalgebra::{Matrix3, SMatrix, UnitQuaternion, Vector3};

use crate::model::mechanism::Marker;
use crate::model::spatial::BodyPose;

/// Child body pose for a joint whose I/J markers coincide.
pub fn child_pose(
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

/// Jacobian blocks of the position residual `p_i(marker) - p_j(marker)`,
/// for a joint whose I/J markers coincide, with respect to the world-frame
/// twists `[v; ω]` of the I and J bodies.
///
/// The fixed body-local marker offsets are mapped to world with each body's
/// current rotation: `skew(R s) = R skew(s) Rᵀ`.
pub fn position_jacobian_blocks(
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

/// Rate of change of a rotation vector (axis-angle `displacement`) given
/// the actual relative angular velocity producing it — the derivative of
/// the `so(3)` log map, independent of joint kind.
pub fn displacement_rate_from_relative_angular_velocity(
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

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::model::mechanism::BodyId;
    use crate::model::spatial::BodyTwist;

    #[test]
    fn displacement_rate_is_inverse_of_relative_angular_velocity() {
        // `relative_angular_velocity` (spherical.rs) is independently
        // verified against a finite difference of the scaled-axis
        // orientation; here `displacement_rate_from_relative_angular_velocity`
        // is checked as its exact inverse, for a displacement rate that is
        // not aligned with the displacement itself (exercises off-axis
        // coupling, not just a scalar rescale along a shared axis).
        use crate::model::coordinates::JointCoordinate;
        use crate::model::mechanism::joint::spherical::SphericalCoordinate;
        use crate::model::motion::JointDisplacement;

        let coordinate = SphericalCoordinate::new(
            UnitQuaternion::identity(),
            JointDisplacement::new(Vector3::new(None, None, None)),
        );
        let displacement = Vector3::new(0.3, -0.5, 0.2);
        let displacement_rate = Vector3::new(0.1, 0.4, -0.3);
        let angular_velocity =
            coordinate.relative_angular_velocity(displacement, displacement_rate);

        let recovered =
            displacement_rate_from_relative_angular_velocity(displacement, angular_velocity);

        assert!((recovered - displacement_rate).norm() < 1.0e-12);
    }

    #[test]
    fn displacement_rate_handles_small_displacement_without_singularity() {
        let displacement = Vector3::new(1.0e-10, -2.0e-10, 3.0e-10);
        let angular_velocity = Vector3::new(0.4, -0.5, 0.6);

        assert!(
            (displacement_rate_from_relative_angular_velocity(displacement, angular_velocity)
                - angular_velocity)
                .norm()
                < 1.0e-9
        );
    }

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

        let child = child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        assert!(
            (parent.marker_position(&i_marker) - child.marker_position(&j_marker)).norm() < 1.0e-12
        );
        assert!(child.orientation().angle_to(&relative_orientation) < 1.0e-12);
    }

    #[test]
    fn position_jacobian_blocks_map_body_twists_to_marker_velocity_difference() {
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

        let (gi, gj) = position_jacobian_blocks(&i_pose, &i_marker, &j_pose, &j_marker);
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
}
