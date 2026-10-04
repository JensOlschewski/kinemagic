use super::mechanism::joint::JointId;
use nalgebra::Vector3;

/// Collection of motions with unique joint identifiers.
pub struct Motions {
    motions: Vec<Motion>,
}

impl Motions {
    pub fn new(motions: Vec<Motion>) -> Self {
        Self { motions }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Motion> {
        self.motions.iter()
    }

    pub fn is_empty(&self) -> bool {
        self.motions.is_empty()
    }
}

impl std::ops::Index<usize> for Motions {
    type Output = Motion;

    fn index(&self, index: usize) -> &Motion {
        &self.motions[index]
    }
}

impl<'a> IntoIterator for &'a Motions {
    type Item = &'a Motion;
    type IntoIter = std::slice::Iter<'a, Motion>;

    fn into_iter(self) -> Self::IntoIter {
        self.motions.iter()
    }
}

/// Prescribed time-dependent displacement for one joint.
///
/// `joint_id` must reference a joint in input mechanism.
pub struct Motion {
    name: String,
    kind: MotionKind,
    joint_id: JointId,
    joint_displacement: JointDisplacement,
}

impl Motion {
    /// Creates prescribed motion for `joint_id`.
    pub fn new(
        name: impl Into<String>,
        kind: MotionKind,
        joint_id: JointId,
        joint_displacement: JointDisplacement,
    ) -> Self {
        Self {
            name: name.into(),
            kind,
            joint_id,
            joint_displacement,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn kind(&self) -> MotionKind {
        self.kind
    }

    pub fn joint_id(&self) -> JointId {
        self.joint_id
    }

    pub fn joint_displacement(&self) -> &JointDisplacement {
        &self.joint_displacement
    }
}

/// Kind of prescribed motion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionKind {
    /// Prescribes joint-coordinate displacement over time.
    JointCoordinates,
}

/// Optional rotational joint-coordinate displacements and constant rates.
///
/// `None` leaves coordinate unspecified. `Some(value)` prescribes coordinate.
/// Rates default to zero when created with [`JointDisplacement::new`].
#[derive(Debug, Clone, Copy)]
pub struct JointDisplacement {
    rotation: Vector3<Option<f64>>,
    rotation_rate: Vector3<Option<f64>>,
}

impl JointDisplacement {
    /// Creates displacement with zero rates for specified coordinates.
    pub fn new(rotation: Vector3<Option<f64>>) -> Self {
        let rotation_rate = rotation.map(|value| value.map(|_| 0.0));
        Self {
            rotation,
            rotation_rate,
        }
    }

    pub fn with_rates(rotation: Vector3<Option<f64>>, rotation_rate: Vector3<Option<f64>>) -> Self {
        Self {
            rotation,
            rotation_rate,
        }
    }

    pub fn rotation(&self) -> &Vector3<Option<f64>> {
        &self.rotation
    }

    pub fn rotation_rate(&self) -> &Vector3<Option<f64>> {
        &self.rotation_rate
    }

    /// Evaluates displacement after `time` using constant rates.
    ///
    /// Each specified coordinate uses `rotation + rotation_rate × time`.
    pub fn at(&self, time: f64) -> Self {
        Self::with_rates(
            Vector3::new(
                self.rotation
                    .x
                    .map(|value| value + self.rotation_rate.x.unwrap_or(0.0) * time),
                self.rotation
                    .y
                    .map(|value| value + self.rotation_rate.y.unwrap_or(0.0) * time),
                self.rotation
                    .z
                    .map(|value| value + self.rotation_rate.z.unwrap_or(0.0) * time),
            ),
            self.rotation_rate,
        )
    }
}
