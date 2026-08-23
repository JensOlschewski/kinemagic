mod coordinates;
mod tree;

use nalgebra::UnitQuaternion;
use thiserror::Error;

use crate::model::{BodyId, Input, JointId, Model};
use coordinates::resolve_joint_coordinates;
use tree::build_kinematic_tree;

pub use coordinates::JointCoordinateError;
pub use tree::KinematicTreeError;

pub struct PreparedProblem {
    input: Input,
    steps: Vec<PreparedStep>,
}

impl PreparedProblem {
    pub fn model(&self) -> &Model {
        self.input.model()
    }

    pub fn steps(&self) -> &[PreparedStep] {
        &self.steps
    }
}

#[derive(Debug)]
pub struct PreparedStep {
    parent_id: BodyId,
    child_id: BodyId,
    joint_id: JointId,
    relative_orientation: UnitQuaternion<f64>,
}

impl PreparedStep {
    pub fn parent_id(&self) -> BodyId {
        self.parent_id
    }

    pub fn child_id(&self) -> BodyId {
        self.child_id
    }

    pub fn joint_id(&self) -> JointId {
        self.joint_id
    }

    pub fn relative_orientation(&self) -> UnitQuaternion<f64> {
        self.relative_orientation
    }
}

pub fn prepare(input: Input) -> Result<PreparedProblem, PrepareError> {
    let tree = build_kinematic_tree(input.model())?;
    let coordinates = resolve_joint_coordinates(&input)?;
    let steps = tree
        .steps()
        .iter()
        .map(|step| {
            let coordinate = coordinates
                .get(step.joint_id)
                .expect("prepared topology references a model joint");

            PreparedStep {
                parent_id: step.parent_id,
                child_id: step.child_id,
                joint_id: step.joint_id,
                relative_orientation: coordinate.relative_orientation(),
            }
        })
        .collect();

    Ok(PreparedProblem { input, steps })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum PrepareError {
    #[error(transparent)]
    Topology(#[from] KinematicTreeError),
    #[error(transparent)]
    Coordinates(#[from] JointCoordinateError),
}
