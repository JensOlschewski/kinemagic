pub mod body;
pub mod joint;

pub use body::{Bodies, Body, BodyError, BodyId};
pub use joint::{Joint, JointError, JointId, JointKind, JointRole, Joints};

use std::collections::HashSet;

use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

/// Validated multibody mechanism.
///
/// Contains bodies and joints connected to ground.
pub struct Mechanism {
    bodies: Bodies,
    joints: Joints,
}

impl Mechanism {
    /// Creates validated mechanism.
    ///
    /// Every joint must reference two existing, distinct bodies.
    /// Mechanism must contain ground and connect every body to ground.
    ///
    /// # Errors
    ///
    /// Returns [`ModelBuildError`] for invalid joint references, self-connecting
    /// joints, missing ground, or bodies disconnected from ground.
    pub fn new(bodies: Bodies, joints: Joints) -> Result<Self, ModelBuildError> {
        for joint in joints.iter() {
            let joint_id = joint.id();
            let i_body_id = joint.i_marker().body_id();
            let j_body_id = joint.j_marker().body_id();

            if !bodies.contains(i_body_id) {
                return Err(ModelBuildError::InvalidJointReference {
                    joint_id,
                    body_id: i_body_id,
                });
            }

            if !bodies.contains(j_body_id) {
                return Err(ModelBuildError::InvalidJointReference {
                    joint_id,
                    body_id: j_body_id,
                });
            }

            if i_body_id == j_body_id {
                return Err(ModelBuildError::SelfConnectingJoint {
                    joint_id,
                    body_id: i_body_id,
                });
            }
        }

        // Ground body is required as traversal root.
        if !bodies.contains(BodyId::GROUND) {
            return Err(ModelBuildError::MissingGround);
        }

        let mut reachable = HashSet::from([BodyId::GROUND]);

        // Expand connected-body set until no new bodies are reachable.
        loop {
            let before = reachable.len();

            for joint in joints.iter() {
                if reachable.contains(&joint.i_marker().body_id()) {
                    reachable.insert(joint.j_marker().body_id());
                }
                if reachable.contains(&joint.j_marker().body_id()) {
                    reachable.insert(joint.i_marker().body_id());
                }
            }

            if reachable.len() == before {
                break;
            }
        }

        for body in bodies.iter() {
            if !reachable.contains(&body.id()) {
                return Err(ModelBuildError::FreeBody(body.id()));
            }
        }

        Ok(Self { bodies, joints })
    }

    /// Returns mechanism bodies.
    pub fn bodies(&self) -> &Bodies {
        &self.bodies
    }

    /// Returns mechanism joints.
    pub fn joints(&self) -> &Joints {
        &self.joints
    }
}

/// Named coordinate frame fixed to a body.
///
/// Joints connect pairs of markers. Marker pose is expressed in its body's
/// local coordinate system.
pub struct Marker {
    name: String,
    body_id: BodyId,
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
}

/// Creates marker fixed to `body_id`.
///
/// `Mechanism::new` validates that `body_id` exists.
impl Marker {
    pub fn new(
        name: impl Into<String>,
        body_id: BodyId,
        position: Vector3<f64>,
        orientation: UnitQuaternion<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            body_id,
            position,
            orientation,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }
}

/// Named point fixed in a body's local coordinate system.
pub struct Point {
    name: String,
    position: Vector3<f64>,
}

impl Point {
    pub fn new(name: impl Into<String>, position: Vector3<f64>) -> Self {
        Self {
            name: name.into(),
            position,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }
}

/// Errors returned when constructing or validating a mechanism.
///
/// A valid mechanism contains ground, uses unique identifiers, connects every
/// body to ground, and has joints between distinct existing bodies.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ModelBuildError {
    #[error(transparent)]
    BodyError(#[from] BodyError),
    #[error(transparent)]
    JointError(#[from] JointError),
    #[error("joint `{joint_id:?}` references an invalid body ID `{body_id:?}`")]
    InvalidJointReference { joint_id: JointId, body_id: BodyId },
    #[error("model does not contain ground body 0")]
    MissingGround,
    #[error("duplicate joint ID `{0:?}`")]
    DuplicateJointId(JointId),
    #[error("body `{0:?}` is not connected to ground")]
    FreeBody(BodyId),
    #[error("joint `{joint_id:?}` connects body `{body_id:?}` to itself")]
    SelfConnectingJoint { joint_id: JointId, body_id: BodyId },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::mechanism::body::*;
    use crate::model::mechanism::joint::*;

    #[test]
    fn build_one_body_model() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "body"),
        ])?;

        let joints = Joints::new(vec![joint(
            JointId::new(1),
            "joint_1",
            BodyId::new(0),
            BodyId::new(1),
        )])?;

        let model = Mechanism::new(bodies, joints)?;

        assert_eq!(model.bodies().iter().count(), 2);
        assert_eq!(model.joints().iter().count(), 1);

        Ok(())
    }

    #[test]
    fn build_two_body_model() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "body_1"),
            body(BodyId::new(2), "body_2"),
        ])?;

        let joints = Joints::new(vec![
            joint(JointId::new(1), "joint_1", BodyId::new(0), BodyId::new(1)),
            joint(JointId::new(2), "joint_2", BodyId::new(1), BodyId::new(2)),
        ])?;

        let model = Mechanism::new(bodies, joints)?;

        assert_eq!(model.bodies().iter().count(), 3);
        assert_eq!(model.joints().iter().count(), 2);

        Ok(())
    }

    #[test]
    fn rejects_missing_i_body() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "body"),
        ])?;

        let joints = Joints::new(vec![joint(
            JointId::new(1),
            "joint_1",
            BodyId::new(99),
            BodyId::new(1),
        )])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::InvalidJointReference { body_id, .. })
                if body_id == BodyId::new(99)
        ));

        Ok(())
    }

    #[test]
    fn rejects_missing_j_body() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "body"),
        ])?;

        let joints = Joints::new(vec![joint(
            JointId::new(1),
            "joint_1",
            BodyId::new(1),
            BodyId::new(99),
        )])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::InvalidJointReference { body_id, .. })
                if body_id == BodyId::new(99)
        ));

        Ok(())
    }

    #[test]
    fn rejects_disconnected_body() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "connected body"),
            body(BodyId::new(2), "disconnected body"),
        ])?;

        let joints = Joints::new(vec![joint(
            JointId::new(1),
            "joint_1",
            BodyId::new(0),
            BodyId::new(1),
        )])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::FreeBody(id))
                if id == BodyId::new(2)
        ));

        Ok(())
    }

    #[test]
    fn rejects_disconnected_bodies_cluster() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "connected body"),
            body(BodyId::new(2), "disconnected body 1"),
            body(BodyId::new(3), "disconnected body 2"),
        ])?;

        let joints = Joints::new(vec![
            joint(JointId::new(1), "joint_1", BodyId::new(0), BodyId::new(1)),
            joint(JointId::new(2), "joint_2", BodyId::new(2), BodyId::new(3)),
        ])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::FreeBody(id))
                if id == BodyId::new(2)
        ));

        Ok(())
    }

    #[test]
    fn rejects_ground_missing() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::new(1), "body 1"),
            body(BodyId::new(2), "body 2"),
        ])?;

        let joints = Joints::new(vec![joint(
            JointId::new(1),
            "joint_1",
            BodyId::new(1),
            BodyId::new(2),
        )])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(model, Err(ModelBuildError::MissingGround)));

        Ok(())
    }

    #[test]
    fn accepts_ground_only_model() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![body(BodyId::GROUND, "ground")])?;
        let joints = Joints::new(vec![])?;

        assert!(Mechanism::new(bodies, joints).is_ok());

        Ok(())
    }

    #[test]
    fn rejects_non_ground_body_without_joints() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "free body"),
        ])?;
        let joints = Joints::new(vec![])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::FreeBody(id))
                if id == BodyId::new(1)
        ));

        Ok(())
    }

    #[test]
    fn rejects_self_connecting_joint() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "body"),
        ])?;

        let joints = Joints::new(vec![joint(
            JointId::new(1),
            "joint_1",
            BodyId::new(1),
            BodyId::new(1),
        )])?;

        let model = Mechanism::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::SelfConnectingJoint {
                joint_id,
                body_id,
            }) if joint_id == JointId::new(1)
                && body_id == BodyId::new(1)
        ));

        Ok(())
    }

    fn body(id: BodyId, name: &str) -> Body {
        Body::new(
            id,
            name,
            Vector3::zeros(),
            UnitQuaternion::identity(),
            vec![],
        )
    }

    fn marker(name: &str, body_id: BodyId) -> Marker {
        Marker::new(name, body_id, Vector3::zeros(), UnitQuaternion::identity())
    }

    fn joint(id: JointId, name: &str, i_body: BodyId, j_body: BodyId) -> Joint {
        Joint::new(
            id,
            name,
            JointKind::Spherical,
            JointRole::Auto,
            marker("i", i_body),
            marker("j", j_body),
        )
    }
}
