use std::error::Error;
use std::fmt;

use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};

#[derive(Clone, Debug)]
pub struct Orientation {
    pub orientation: UnitQuaternion<f64>,
}

impl Orientation {
    pub fn new(orientation: UnitQuaternion<f64>) -> Self {
        Self { orientation }
    }

    pub fn rotation_matrix(&self) -> Rotation3<f64> {
        self.orientation.to_rotation_matrix()
    }

    pub fn from_spec(
        spec: &OrientationSpec,
        resolve_point: impl Fn(&str) -> Option<Vector3<f64>>,
    ) -> Result<Self, BuildOrientationError> {
        let orientation = match spec {
            OrientationSpec::Euler { euler_angles } => from_euler_zxz_degrees(*euler_angles),
            OrientationSpec::Quaternion { quaternion } => from_quaternion(*quaternion)?,
            OrientationSpec::TwoPoints {
                p1,
                p2,
                axis,
                helpvec,
                axisortho,
            } => {
                let p1 = resolve_named_point(p1, &resolve_point)?;
                let p2 = resolve_named_point(p2, &resolve_point)?;

                from_two_points(p1, p2, *axis, *helpvec, *axisortho)?
            }
            OrientationSpec::ThreePoints {
                p1,
                p2,
                p3,
                axis12,
                axis13,
            } => {
                let p1 = resolve_named_point(p1, &resolve_point)?;
                let p2 = resolve_named_point(p2, &resolve_point)?;
                let p3 = resolve_named_point(p3, &resolve_point)?;

                from_three_points(p1, p2, p3, *axis12, *axis13)?
            }
            OrientationSpec::LowestEnergy {
                stiffness,
                euler_angles,
                fconstr,
                defaxis,
                helpvec,
                axisortho,
            } => from_lowest_energy(
                *stiffness,
                *euler_angles,
                *fconstr,
                *defaxis,
                *helpvec,
                *axisortho,
            )?,
        };

        Ok(Self::new(orientation))
    }
}

// Helper function to convert Euler angles (z-x-z convention)
// in degrees to a UnitQuaternion
fn from_euler_zxz_degrees(euler_angles: [f64; 3]) -> UnitQuaternion<f64> {
    let [z1, x, z2] = euler_angles.map(f64::to_radians);

    let q_z1 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), z1);
    let q_x = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), x);
    let q_z2 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), z2);

    q_z1 * q_x * q_z2
}

// Helper function to convert a quaternion to a UnitQuaternion
fn from_quaternion(quaternion: [f64; 4]) -> Result<UnitQuaternion<f64>, BuildOrientationError> {
    let [e0, e1, e2, e3] = quaternion;
    let q = Quaternion::new(e0, e1, e2, e3);

    UnitQuaternion::try_new(q, 0.0).ok_or(BuildOrientationError::ZeroQuaternion)
}

// Build orientation from two points.
// The first axis is the normalized direction from p1 to p2.
// The second axis is the perpendicular component of helpvec relative to p1->p2.
// The third axis is the cross product of the first two axes.
// axis and axisortho define how these directions are assigned to X/Y/Z.
fn from_two_points(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    axis: Axis,
    helpvec: [f64; 3],
    axisortho: Axis,
) -> Result<UnitQuaternion<f64>, BuildOrientationError> {
    let a = (p2 - p1)
        .try_normalize(1e-12)
        .ok_or(BuildOrientationError::ZeroVector("p1->p2"))?;

    let h = Vector3::from(helpvec);

    let b = (h - a * h.dot(&a))
        .try_normalize(1.0e-12)
        .ok_or(BuildOrientationError::ParallelVectors)?;

    Ok(UnitQuaternion::from_rotation_matrix(
        &Rotation3::from_matrix_unchecked(Axis::matrix_from_axes(axis, axisortho, a, b)?),
    ))
}

// Build orientation from three points.
// The first axis (a) is the normalized direction from p1 to p2.
// The second axis (b) is the perpendicular component of helpvec relative to p1->p2.
// The third axis (c) is the cross product of the first two axes.
// axis and axisortho define how these directions are assigned to X/Y/Z.
fn from_three_points(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    p3: Vector3<f64>,
    axis12: Axis,
    axis13: Axis,
) -> Result<UnitQuaternion<f64>, BuildOrientationError> {
    let a = (p2 - p1)
        .try_normalize(1e-12)
        .ok_or(BuildOrientationError::ZeroVector("p1->p2"))?;

    let b = (p3 - p1)
        .try_normalize(1e-12)
        .ok_or(BuildOrientationError::ZeroVector("p1->p3"))?;

    Ok(UnitQuaternion::from_rotation_matrix(
        &Rotation3::from_matrix_unchecked(Axis::matrix_from_axes(axis12, axis13, a, b)?),
    ))
}
// from_lowest_energy
fn from_lowest_energy(
    stiffness: [f64; 6],
    euler_angles: [f64; 3],
    fconstr: [f64; 3],
    defaxis: Axis,
    helpvec: [f64; 3],
    axisortho: Axis,
) -> Result<UnitQuaternion<f64>, BuildOrientationError> {
    let local_compliance = Matrix3::from_diagonal(&Vector3::new(
        1.0 / stiffness[0],
        1.0 / stiffness[1],
        1.0 / stiffness[2],
    ));

    let rotation = from_euler_zxz_degrees(euler_angles)
        .to_rotation_matrix()
        .into_inner();

    let global_compliance = rotation * local_compliance * rotation.transpose();
    let deformation_direction = global_compliance * Vector3::from(fconstr);

    let a = deformation_direction
        .try_normalize(1e-12)
        .ok_or(BuildOrientationError::ZeroVector("deformation direction"))?;

    let h = Vector3::from(helpvec);

    let b = h
        .cross(&a)
        .try_normalize(1e-12)
        .ok_or(BuildOrientationError::ParallelVectors)?;

    Ok(UnitQuaternion::from_rotation_matrix(
        &Rotation3::from_matrix_unchecked(Axis::matrix_from_axes(defaxis, axisortho, a, b)?),
    ))
}

// Helper function to resolve Vector3
fn resolve_named_point(
    name: &str,
    resolve_point: impl Fn(&str) -> Option<Vector3<f64>>,
) -> Result<Vector3<f64>, BuildOrientationError> {
    resolve_point(name).ok_or_else(|| BuildOrientationError::MissingPoint(name.to_string()))
}

#[derive(Clone, Debug)]
pub enum OrientationSpec {
    /// Orientation specified by Euler angles in degrees
    /// z-x-z convention.
    Euler { euler_angles: [f64; 3] },

    /// Orientation specified by a quaternion
    /// e0 is the scalar part, e1..e3 the vector part.
    Quaternion { quaternion: [f64; 4] },

    /// First axis points from p1 to p2.
    /// A second axis is built from `helpvec`.
    TwoPoints {
        p1: String,
        p2: String,
        axis: Axis,
        helpvec: [f64; 3],
        axisortho: Axis,
    },

    /// Two axes defined by p1->p2 and p1->p3.
    ThreePoints {
        p1: String,
        p2: String,
        p3: String,
        axis12: Axis,
        axis13: Axis,
    },

    /// Bushing orientation that minimizes potential energy for a force constraint.
    LowestEnergy {
        stiffness: [f64; 6],
        euler_angles: [f64; 3],
        fconstr: [f64; 3],
        defaxis: Axis,
        helpvec: [f64; 3],
        axisortho: Axis,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    // Builds a right-handed basis matrix from two named axes and their vectors.
    // `axis_a` and `axis_b` define which matrix columns receive `vec_a` and `vec_b`.
    // The remaining column is filled with the cross product in the order needed to
    // preserve the X/Y/Z axis orientation.
    // Returns an error if both vectors are assigned to the same axis.
    pub fn matrix_from_axes(
        axis_a: Axis,
        axis_b: Axis,
        vec_a: Vector3<f64>,
        vec_b: Vector3<f64>,
    ) -> Result<Matrix3<f64>, BuildOrientationError> {
        let cols = match (axis_a, axis_b) {
            (Axis::X, Axis::Y) => [vec_a, vec_b, vec_a.cross(&vec_b)],
            (Axis::X, Axis::Z) => [vec_a, vec_b.cross(&vec_a), vec_b],
            (Axis::Y, Axis::X) => [vec_b, vec_a, vec_b.cross(&vec_a)],
            (Axis::Y, Axis::Z) => [vec_a.cross(&vec_b), vec_a, vec_b],
            (Axis::Z, Axis::X) => [vec_b, vec_a.cross(&vec_b), vec_a],
            (Axis::Z, Axis::Y) => [vec_b.cross(&vec_a), vec_b, vec_a],
            (first, second) if first == second => {
                return Err(BuildOrientationError::SameAxis { first, second });
            }
            _ => unreachable!(),
        };
        Ok(Matrix3::from_columns(&cols))
    }
}

#[derive(Debug)]
pub enum BuildOrientationError {
    MissingPoint(String),
    SameAxis { first: Axis, second: Axis },
    ZeroVector(&'static str),
    ParallelVectors,
    ZeroQuaternion,
}

impl fmt::Display for BuildOrientationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingPoint(point) => {
                write!(f, "orientation references missing point '{point}'")
            }
            Self::SameAxis { first, second } => {
                write!(
                    f,
                    "orientation axes must be different: {first:?} and {second:?}"
                )
            }
            Self::ZeroVector(name) => write!(f, "orientation vector '{name}' must not be zero"),
            Self::ParallelVectors => write!(f, "orientation vectors must not be parallel"),
            Self::ZeroQuaternion => write!(f, "orientation quaternion must not be zero"),
        }
    }
}

impl Error for BuildOrientationError {}
