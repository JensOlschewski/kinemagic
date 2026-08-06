use crate::kinematics::coordinates::JointCoordinates;
use crate::model::JointKey;

pub struct MotionSpec {
    name: String,
    kind: MotionKind,
}

pub enum MotionKind {
    JointCoordinates {
        key: JointKey,
        coordinates: JointCoordinates,
    },
}

impl MotionSpec {
    pub fn new(name: String, kind: MotionKind) -> Self {
        Self { name, kind }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn kind(&self) -> &MotionKind {
        &self.kind
    }
}
