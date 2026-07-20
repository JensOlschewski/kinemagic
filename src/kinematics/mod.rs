pub mod forward;
pub mod state;

pub mod spherical;

use crate::model::{BodyId, BodyPose, JointKind, JointKey, JointMarker};
use nalgebra::{Matrix3, SMatrix, SVector, Vector3, stack};

pub struct ImplicitPosition<const CONSTRAINTS: usize> {
    /// Position level: g_ij(q_i, q_j) = 0
    pub position_residual: SVector<f64, CONSTRAINTS>,
}

pub struct ImplicitVelocity<const CONSTRAINTS: usize> {
    /// Velocity level: Gi * v_i + Gj * v_j = 0
    pub gi: SMatrix<f64, CONSTRAINTS, 6>,
    pub gj: SMatrix<f64, CONSTRAINTS, 6>,
}

pub struct ExplicitVelocity<const DOFS: usize> {
    /// Rigid transformation from i to body j.
    pub b_ji: SMatrix<f64, 6, 6>,
    /// Joint-specific Jacobian mapping η_j to relative velocity.
    pub hj: SMatrix<f64, 6, DOFS>,
}

pub struct JointKinematicsInput {
    /// Global lever arm from body origin O_i to marker P_i.
    pub si: Vector3<f64>,
    /// Global lever arm from body origin O_j to marker P_j.
    pub sj: Vector3<f64>,
    /// Global position of marker P_i.
    pub point_i: Vector3<f64>,
    /// Global position of marker P_j.
    pub point_j: Vector3<f64>,
    /// Vector from O_j to O_i.
    pub r_ji: Vector3<f64>,
}

impl JointKinematicsInput {
    pub fn from_poses_and_markers(
        local_marker_i: &JointMarker,
        local_marker_j: &JointMarker,
        pose_i: &BodyPose,
        pose_j: &BodyPose,
    ) -> Self {
        let point_i = local_marker_i.global_position(pose_i);
        let point_j = local_marker_j.global_position(pose_j);

        let si = point_i - pose_i.position;
        let sj = point_j - pose_j.position;

        let r_ji = pose_i.position - pose_j.position;

        Self {
            point_i,
            point_j,
            si,
            sj,
            r_ji,
        }
    }
}

pub fn shift_matrix(r_ji: &Vector3<f64>) -> SMatrix<f64, 6, 6> {
    stack![
        Matrix3::identity(), &r_ji.cross_matrix();
        Matrix3::zeros(), Matrix3::identity()
    ]
}

#[derive(Debug)]
pub enum KinematicsError {
    MissingBody(BodyId),
    UnsupportedJoint(JointKind),
    MissingJointCoordinates(JointKey),
    InvalidPrimaryTree,
}

impl std::error::Error for KinematicsError {}

impl std::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingBody(id) => write!(f, "missing body {}", id.0),
            Self::UnsupportedJoint(kind) => write!(f, "unsupported joint type: {kind:?}"),
            Self::MissingJointCoordinates(key) => write!(f, "missing joint coordinates for joint: {key:?}"),
            Self::InvalidPrimaryTree => write!(f, "invalid tree"),
        }
    }
}
