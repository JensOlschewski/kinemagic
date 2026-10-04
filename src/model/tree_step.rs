use nalgebra::{UnitQuaternion, Vector3};

use super::mechanism::{BodyId, JointId};
use super::topology::TraversalDirection;
use crate::model::mechanism::joint::spherical::SphericalCoordinate;

/// Spanning-tree edge together with its joint coordinate.
#[derive(Debug)]
pub struct TreeStep {
    parent_body_id: BodyId,
    child_body_id: BodyId,
    joint_id: JointId,
    joint_coordinate: SphericalCoordinate,
    direction: TraversalDirection,
}

impl TreeStep {
    pub fn parent_body_id(&self) -> BodyId {
        self.parent_body_id
    }

    pub fn child_body_id(&self) -> BodyId {
        self.child_body_id
    }

    pub fn joint_id(&self) -> JointId {
        self.joint_id
    }

    pub fn joint_coordinate(&self) -> SphericalCoordinate {
        self.joint_coordinate
    }

    pub fn direction(&self) -> TraversalDirection {
        self.direction
    }

    pub fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.joint_coordinate.relative_orientation()
    }

    pub fn traversal_relative_orientation(&self, candidate: Vector3<f64>) -> UnitQuaternion<f64> {
        self.traversal_relative_orientation_at(0.0, candidate)
    }

    pub fn traversal_relative_orientation_at(
        &self,
        time: f64,
        candidate: Vector3<f64>,
    ) -> UnitQuaternion<f64> {
        let orientation = self
            .joint_coordinate
            .relative_orientation_at(time, candidate);

        match self.direction {
            TraversalDirection::IToJ => orientation,
            TraversalDirection::JToI => orientation.inverse(),
        }
    }

    pub fn traversal_relative_angular_velocity(
        &self,
        displacement: Vector3<f64>,
        displacement_rate: Vector3<f64>,
    ) -> Vector3<f64> {
        match self.direction {
            TraversalDirection::IToJ => self
                .joint_coordinate
                .relative_angular_velocity(displacement, displacement_rate),
            TraversalDirection::JToI => self
                .joint_coordinate
                .reverse_relative_angular_velocity(displacement, displacement_rate),
        }
    }
}

impl TreeStep {
    pub(crate) fn new(
        parent_body_id: BodyId,
        child_body_id: BodyId,
        joint_id: JointId,
        joint_coordinate: SphericalCoordinate,
        direction: TraversalDirection,
    ) -> Self {
        Self {
            parent_body_id,
            child_body_id,
            joint_id,
            joint_coordinate,
            direction,
        }
    }
}
