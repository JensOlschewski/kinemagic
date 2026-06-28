use std::collections::BTreeMap;

use nalgebra::Vector3;

#[derive(Debug, Clone, PartialEq)]
pub struct Hardpoints {
    points: HashMap<String, Vector3<f64>>,
}
