use std::collections::BTreeMap;

use nalgebra::Vector3;

#[derive(Debug, Clone, PartialEq)]
pub struct Hardpoints {
    points: BTreeMap<String, Vector3<f64>>,
}

impl Hardpoints {
    pub fn new(points: BTreeMap<String, Vector3<f64>>) -> Self {
        Self { points }
    }

    pub fn get(&self, name: &str) -> Option<&Vector3<f64>> {
        self.points.get(name)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&String, &Vector3<f64>)> {
        self.points.iter()
    }
}
