use std::collections::BTreeMap;
use nalgebra::UnitQuaternion;

pub struct MotionSpec {
    pub joint_coordinates: BTreeMap<String, PrescribedJointCoordinates>,
}

pub enum PrescribedJointCoordinates {
    Spherical {
        relative_orientation: UnitQuaternion<f64>,
    },
}
