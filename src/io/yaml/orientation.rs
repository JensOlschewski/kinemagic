use serde::Deserialize;

use crate::model::{Axis, OrientationSpec};

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "method", rename_all = "kebab-case")]
pub enum YamlOrientation {
    // Orientation specified by Euler angles (z,x,z convention)
    Euler {
        euler_angles: [f64; 3],
    },
    // Orientation specified by a quaternion (e0, e1, e2, e3),
    // e0 is the scalar part, e1, e2, e3 are the vector part
    Quaternion {
        quaternion: [f64; 4],
    },
    // Orientation specified by two points and an axis
    // first axis along p1 and p2,
    // second axis orthogonal to the first axis and helpvec,
    // third axis orthogonal to the first two axes
    TwoPoints {
        p1: String,
        p2: String,
        axis: YamlAxis,
        helpvec: [f64; 3],
        axisortho: YamlAxis,
    },
    // Orientation specified by three points and two axes
    ThreePoints {
        p1: String,
        p2: String,
        p3: String,
        axis12: YamlAxis,
        axis13: YamlAxis,
    },
    LowestEnergy {
        stiffness: [f64; 6],
        euler_angles: [f64; 3],
        fconstr: [f64; 3],
        defaxis: YamlAxis,
        helpvec: [f64; 3],
        axisortho: YamlAxis,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum YamlAxis {
    X,
    Y,
    Z,
}

impl From<YamlOrientation> for OrientationSpec {
    fn from(value: YamlOrientation) -> Self {
        match value {
            YamlOrientation::Euler { euler_angles } => Self::Euler { euler_angles },
            YamlOrientation::Quaternion { quaternion } => Self::Quaternion { quaternion },
            YamlOrientation::TwoPoints {
                p1,
                p2,
                axis,
                helpvec,
                axisortho,
            } => Self::TwoPoints {
                p1,
                p2,
                axis: axis.into(),
                helpvec,
                axisortho: axisortho.into(),
            },
            YamlOrientation::ThreePoints {
                p1,
                p2,
                p3,
                axis12,
                axis13,
            } => Self::ThreePoints {
                p1,
                p2,
                p3,
                axis12: axis12.into(),
                axis13: axis13.into(),
            },
            YamlOrientation::LowestEnergy {
                stiffness,
                euler_angles,
                fconstr,
                defaxis,
                helpvec,
                axisortho,
            } => Self::LowestEnergy {
                stiffness,
                euler_angles,
                fconstr,
                defaxis: defaxis.into(),
                helpvec,
                axisortho: axisortho.into(),
            },
        }
    }
}

impl From<YamlAxis> for Axis {
    fn from(value: YamlAxis) -> Self {
        match value {
            YamlAxis::X => Self::X,
            YamlAxis::Y => Self::Y,
            YamlAxis::Z => Self::Z,
        }
    }
}
