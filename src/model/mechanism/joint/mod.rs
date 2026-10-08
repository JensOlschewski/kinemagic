pub mod geometry;
pub mod revolute;
pub mod spherical;

use std::collections::HashSet;

use thiserror::Error;

use super::Marker;

/// Collection of joints with unique identifiers.
pub struct Joints {
    joints: Vec<Joint>,
}
impl Joints {
    //// Creates joint collection.
    ///
    /// # Errors
    ///
    /// Returns [`ModelBuildError::DuplicateJointId`] when multiple joints share an ID.
    pub fn new(joints: Vec<Joint>) -> Result<Self, JointError> {
        let mut ids = HashSet::new();

        for joint in &joints {
            if !ids.insert(joint.id()) {
                return Err(JointError::DuplicateJointId(joint.id()));
            }
        }

        Ok(Self { joints })
    }

    /// Iterates over joints in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = &Joint> {
        self.joints.iter()
    }

    /// Returns joint with given ID, if present.
    pub fn get(&self, id: JointId) -> Option<&Joint> {
        self.iter().find(|joint| joint.id() == id)
    }

    /// Returns `true` when collection contains given joint ID.
    pub fn contains(&self, id: JointId) -> bool {
        self.get(id).is_some()
    }
}

/// Kinematic joint constraint between markers fixed on two distinct bodies.
///
/// I/J marker order identifies joint endpoints. It does not define tree
/// traversal direction.
pub struct Joint {
    id: JointId,
    name: String,
    kind: JointKind,
    role: JointRole,
    i_marker: Marker,
    j_marker: Marker,
}

impl Joint {
    /// Creates joint from endpoint markers.
    ///
    /// `Mechanism::new` validates marker body references and rejects joints whose
    /// markers belong to same body.
    pub fn new(
        id: JointId,
        name: impl Into<String>,
        kind: JointKind,
        role: JointRole,
        i_marker: Marker,
        j_marker: Marker,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            kind,
            role,
            i_marker,
            j_marker,
        }
    }

    pub fn id(&self) -> JointId {
        self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn kind(&self) -> JointKind {
        self.kind
    }

    pub fn role(&self) -> JointRole {
        self.role
    }

    pub fn i_marker(&self) -> &Marker {
        &self.i_marker
    }

    pub fn j_marker(&self) -> &Marker {
        &self.j_marker
    }
}

/// Joint kinematic constraint.
///
/// `#[non_exhaustive]` allows future joint types. Downstream matches need `_`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum JointKind {
    /// Allows unrestricted relative rotation between connected bodies.
    Spherical,
    /// Allows relative rotation about a single axis between connected bodies.
    Revolute,
}

/// Hint controlling joint selection during topology construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JointRole {
    /// Select role during topology construction.
    Auto,
    /// Prefer joint for primary topology role.
    Primary,
    /// Prefer joint for secondary topology role.
    Secondary,
}

/// Stable Identifier for a joint in the mechanism
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
pub struct JointId(u32);

impl JointId {
    pub fn new(value: u32) -> Self {
        Self(value)
    }
}

/// Errors returned when constructing or validating a mechanism.
///
/// A valid mechanism contains ground, uses unique identifiers, connects every
/// body to ground, and has joints between distinct existing bodies.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum JointError {
    #[error("duplicate joint ID `{0:?}`")]
    DuplicateJointId(JointId),
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::model::mechanism::{BodyId, Marker};
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn build_joint() {
        let i_body = BodyId::new(1);
        let j_body = BodyId::new(2);
        let i_marker = Marker::new(
            "i-marker",
            i_body,
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        let j_marker = Marker::new(
            "j-marker",
            j_body,
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        let joint = Joint::new(
            JointId::new(1),
            "joint",
            JointKind::Spherical,
            JointRole::Auto,
            i_marker,
            j_marker,
        );

        assert_eq!(joint.id(), JointId::new(1));
        assert_eq!(joint.name(), "joint");
        assert_eq!(joint.kind(), JointKind::Spherical);
        assert_eq!(joint.role(), JointRole::Auto);
        assert_eq!(joint.i_marker().body_id(), i_body);
        assert_eq!(joint.j_marker().body_id(), j_body);
        assert_eq!(joint.i_marker().name(), "i-marker");
        assert_eq!(joint.j_marker().name(), "j-marker");
    }

    #[test]
    fn rejects_duplicate_joint_id() -> Result<(), JointError> {
        let joints = Joints::new(vec![
            create_joint(JointId::new(1), "joint_1", BodyId::new(0), BodyId::new(1)),
            create_joint(JointId::new(1), "joint_1", BodyId::new(1), BodyId::new(2)),
        ]);

        assert!(matches!(
        joints,
        Err(JointError::DuplicateJointId(id)) if id == JointId::new(1)
        ));

        Ok(())
    }

    fn create_joint(id: JointId, name: &str, i_body: BodyId, j_body: BodyId) -> Joint {
        Joint::new(
            id,
            name,
            JointKind::Spherical,
            JointRole::Auto,
            marker("i", i_body),
            marker("j", j_body),
        )
    }

    fn marker(name: &str, body_id: BodyId) -> Marker {
        Marker::new(name, body_id, Vector3::zeros(), UnitQuaternion::identity())
    }
}
