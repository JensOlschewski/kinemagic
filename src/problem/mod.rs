pub mod coordinates;
pub mod tree;

use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

use crate::model::{BodyId, Input, JointId, Model};
use coordinates::resolve_joint_coordinates;
pub use tree::{TraversalDirection, build_kinematic_topology};

pub use coordinates::{JointCoordinate, JointCoordinateError};
pub use tree::KinematicTopologyError;

pub struct PreparedProblem {
    input: Input,
    joint_coordinates: coordinates::JointCoordinates,
    tree_edges: Vec<PreparedTreeEdge>,
    closure_joint_ids: Vec<JointId>,
}

impl PreparedProblem {
    pub fn model(&self) -> &Model {
        self.input.model()
    }

    pub fn tree_edges(&self) -> &[PreparedTreeEdge] {
        &self.tree_edges
    }

    pub fn closure_joint_ids(&self) -> &[JointId] {
        &self.closure_joint_ids
    }

    pub fn joint_coordinate(&self, joint_id: JointId) -> Option<&JointCoordinate> {
        self.joint_coordinates.get(joint_id)
    }
}

#[derive(Debug)]
pub struct PreparedTreeEdge {
    parent_body_id: BodyId,
    child_body_id: BodyId,
    joint_id: JointId,
    joint_coordinate: JointCoordinate,
    direction: TraversalDirection,
}

impl PreparedTreeEdge {
    pub fn parent_body_id(&self) -> BodyId {
        self.parent_body_id
    }

    pub fn child_body_id(&self) -> BodyId {
        self.child_body_id
    }

    pub fn joint_id(&self) -> JointId {
        self.joint_id
    }

    pub fn joint_coordinate(&self) -> JointCoordinate {
        self.joint_coordinate
    }

    pub fn direction(&self) -> TraversalDirection {
        self.direction
    }

    pub fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.joint_coordinate.relative_orientation()
    }

    pub fn traversal_relative_orientation(&self, candidate: Vector3<f64>) -> UnitQuaternion<f64> {
        let orientation = self.joint_coordinate.relative_orientation_for(candidate);

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

pub fn prepare(input: Input) -> Result<PreparedProblem, PrepareError> {
    let topology = build_kinematic_topology(input.model())?;
    let coordinates = resolve_joint_coordinates(&input)?;

    let tree_edges = topology
        .tree_edges()
        .iter()
        .map(|tree_edge| {
            let coordinate = coordinates
                .get(tree_edge.joint_id())
                .expect("prepared topology references a model joint");

            PreparedTreeEdge {
                parent_body_id: tree_edge.parent_body_id(),
                child_body_id: tree_edge.child_body_id(),
                joint_id: tree_edge.joint_id(),
                joint_coordinate: *coordinate,
                direction: tree_edge.direction(),
            }
        })
        .collect();

    Ok(PreparedProblem {
        input,
        joint_coordinates: coordinates,
        tree_edges,
        closure_joint_ids: topology.closure_joint_ids().to_vec(),
    })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum PrepareError {
    #[error(transparent)]
    Topology(#[from] KinematicTopologyError),
    #[error(transparent)]
    Coordinates(#[from] JointCoordinateError),
}
