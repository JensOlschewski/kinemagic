use nalgebra::{Matrix3, SMatrix, UnitQuaternion, stack};

use super::{ExplicitVelocity, ImplicitPosition, ImplicitVelocity, JointKinematicsInput};
use crate::kinematics::shift_matrix;
use crate::model::{JointEndpoint, BodyPose};

pub struct SphericalCoordinates {
    pub relative_orientation: UnitQuaternion<f64>,
}

pub fn implicit_position(input: &JointKinematicsInput) -> ImplicitPosition<3> {
    ImplicitPosition {
        position_residual: input.point_j - input.point_i,
    }
}

pub fn implicit_velocity(input: &JointKinematicsInput) -> ImplicitVelocity<3> {
    let gi: SMatrix<f64, 3, 6> = stack![-Matrix3::identity(), &input.si.cross_matrix()];
    let gj: SMatrix<f64, 3, 6> = stack![Matrix3::identity(), -&input.sj.cross_matrix()];

    ImplicitVelocity { gi, gj }
}

pub fn explicit_position(
    parent_pose: &BodyPose,
    point_i: &JointEndpoint,
    point_j: &JointEndpoint,
    coordinates: &SphericalCoordinates,
) -> BodyPose {
    let marker_i_orientation = point_i.local_marker.to_unit_quaternion();
    let marker_j_orientation = point_j.local_marker.to_unit_quaternion();

    let child_orientation = parent_pose.orientation
            * marker_i_orientation
            * coordinates.relative_orientation
            * marker_j_orientation.inverse();

    let joint_position = parent_pose.local_to_global(point_i.local_marker.local_position);

    let child_position =
        joint_position - child_orientation.transform_vector(&point_j.local_marker.local_position);

    BodyPose::new(child_position, child_orientation)
}

pub fn explicit_velocity(input: &JointKinematicsInput) -> ExplicitVelocity<3> {
    let dj_skew = -&input.sj.cross_matrix();

    let b_ji = shift_matrix(&input.r_ji);

    let hj: SMatrix<f64, 6, 3> = stack![
        -dj_skew;
        Matrix3::identity()
    ];

    ExplicitVelocity { b_ji, hj }
}
