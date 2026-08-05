pub mod bodies;
pub mod hardpoints;
pub mod joints;
pub mod motion;
pub mod orientation;
pub mod side;

pub use bodies::*;
pub use hardpoints::Hardpoints;
pub use joints::*;
pub use orientation::*;
pub use side::*;

pub struct Model {
    bodies: Bodies,
    joints: Joints,
}

impl Model {
    pub fn new(bodies: Bodies, joints: Joints) -> Self {
        Self { bodies, joints }
    }

    pub fn bodies(&self) -> &Bodies {
        &self.bodies
    }

    pub fn joints(&self) -> &Joints {
        &self.joints
    }
}
