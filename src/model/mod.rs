use std::collections::HashSet;

use nalgebra::{Rotation3, UnitQuaternion, Vector3};
use thiserror::Error;

pub struct Model {
    bodies: Bodies,
    joints: Joints,
}

impl Model {
    pub fn new(bodies: Bodies, joints: Joints) -> Result<Self, ModelBuildError> {
        for joint in joints.iter() {
            if !bodies.contains(joint.i_body) {
                return Err(ModelBuildError::InvalidJointReference {
                    joint_id: joint.id,
                    body_id: joint.i_body,
                });
            }

            if !bodies.contains(joint.j_body) {
                return Err(ModelBuildError::InvalidJointReference {
                    joint_id: joint.id,
                    body_id: joint.j_body,
                });
            }
        }

        // BodyId(0) is reserved for ground
        if !bodies.contains(BodyId::GROUND) {
            return Err(ModelBuildError::MissingGround);
        }

        // Check if free body exists
        let mut reachable = HashSet::from([BodyId::GROUND]);

        loop {
            let before = reachable.len();

            for joint in joints.iter() {
                if reachable.contains(&joint.i_body) {
                    reachable.insert(joint.j_body);
                }
                if reachable.contains(&joint.j_body) {
                    reachable.insert(joint.i_body);
                }
            }

            if reachable.len() == before {
                break;
            }
        }

        for body in bodies.iter() {
            if !reachable.contains(&body.id) {
                return Err(ModelBuildError::FreeBody(body.id));
            }
        }

        Ok(Self { bodies, joints })
    }
    pub fn bodies(&self) -> &Bodies {
        &self.bodies
    }

    pub fn joints(&self) -> &Joints {
        &self.joints
    }
}

pub struct Bodies {
    bodies: Vec<Body>,
}

impl Bodies {
    pub fn new(bodies: Vec<Body>) -> Result<Self, ModelBuildError> {
        let mut ids = HashSet::new();

        for body in &bodies {
            if !ids.insert(body.id()) {
                return Err(ModelBuildError::DuplicateBodyId(body.id()));
            }
        }

        Ok(Self { bodies })
    }

    pub fn get(&self, id: BodyId) -> Option<&Body> {
        self.bodies.iter().find(|body| body.id() == id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &Body> {
        self.bodies.iter()
    }

    pub fn contains(&self, id: BodyId) -> bool {
        self.get(id).is_some()
    }
}

pub struct Joints {
    joints: Vec<Joint>,
}

impl Joints {
    pub fn new(joints: Vec<Joint>) -> Result<Self, ModelBuildError> {
        let mut ids = HashSet::new();

        for joint in &joints {
            if !ids.insert(joint.id()) {
                return Err(ModelBuildError::DuplicateJointId(joint.id()));
            }
        }

        Ok(Self { joints })
    }

    pub fn iter(&self) -> impl Iterator<Item = &Joint> {
        self.joints.iter()
    }
}

pub struct Body {
    id: BodyId,
    name: String,
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
    points: Vec<Point>,
}

impl Body {
    pub fn new(
        id: BodyId,
        name: impl Into<String>,
        position: Vector3<f64>,
        orientation: UnitQuaternion<f64>,
        points: Vec<Point>,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            position,
            orientation,
            points,
        }
    }

    pub fn id(&self) -> BodyId {
        self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }

    pub fn points(&self) -> &[Point] {
        &self.points
    }
}

pub struct Joint {
    id: JointId,
    name: String,
    kind: JointKind,
    i_marker: Marker,
    j_marker: Marker,
    i_body: BodyId,
    j_body: BodyId,
}

impl Joint {
    pub fn new(
        id: JointId,
        name: impl Into<String>,
        kind: JointKind,
        i_marker: Marker,
        j_marker: Marker,
        i_body: BodyId,
        j_body: BodyId,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            kind,
            i_marker,
            j_marker,
            i_body,
            j_body,
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

    pub fn i_marker(&self) -> &Marker {
        &self.i_marker
    }

    pub fn j_marker(&self) -> &Marker {
        &self.j_marker
    }

    pub fn i_body(&self) -> BodyId {
        self.i_body
    }

    pub fn j_body(&self) -> BodyId {
        self.j_body
    }
}

pub struct Marker {
    name: String,
    position: Vector3<f64>,
    orientation: Rotation3<f64>,
}

impl Marker {
    pub fn new(
        name: impl Into<String>,
        position: Vector3<f64>,
        orientation: Rotation3<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            position,
            orientation,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn orientation(&self) -> Rotation3<f64> {
        self.orientation
    }
}

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum JointKind {
    Spherical,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct BodyId(u32);

impl BodyId {
    // BodyId(0) is reserved for ground.
    pub const GROUND: Self = Self(0);

    pub fn new(value: u32) -> Self {
        Self(value)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct JointId(u32);

impl JointId {
    pub fn new(value: u32) -> Self {
        Self(value)
    }
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ModelBuildError {
    #[error("joint `{joint_id:?}` references an invalid body ID `{body_id:?}`")]
    InvalidJointReference { joint_id: JointId, body_id: BodyId },
    #[error("model does not contain ground body 0")]
    MissingGround,
    #[error("duplicate body ID `{0:?}`")]
    DuplicateBodyId(BodyId),
    #[error("duplicate joint ID `{0:?}`")]
    DuplicateJointId(JointId),
    #[error("body `{0:?}` is not connected to ground")]
    FreeBody(BodyId),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_body() {
        let id = BodyId::new(1);
        let position = Vector3::new(1.0, 2.0, 3.0);
        let orientation = UnitQuaternion::identity();

        let body = Body::new(id, "body", position, orientation, vec![]);

        assert_eq!(body.id(), id);
        assert_eq!(body.name(), "body");
        assert_eq!(body.position(), position);
        assert_eq!(body.orientation(), orientation);
        assert!(body.points().is_empty());
    }

    #[test]
    fn build_point() {
        let position = Vector3::new(1.0, 2.0, 3.0);
        let point = Point::new("point", position);

        assert_eq!(point.name(), "point");
        assert_eq!(point.position(), position);
    }

    #[test]
    fn build_marker() {
        let position = Vector3::new(1.0, 2.0, 3.0);
        let orientation = Rotation3::identity();
        let marker = Marker::new("marker", position, orientation);

        assert_eq!(marker.name(), "marker");
        assert_eq!(marker.position(), position);
        assert_eq!(marker.orientation(), orientation);
    }

    #[test]
    fn build_joint() {
        let i_marker = Marker::new("i-marker", Vector3::zeros(), Rotation3::identity());
        let j_marker = Marker::new("j-marker", Vector3::zeros(), Rotation3::identity());
        let i_body = BodyId::new(1);
        let j_body = BodyId::new(2);
        let joint = Joint::new(
            JointId::new(1),
            "joint",
            JointKind::Spherical,
            i_marker,
            j_marker,
            i_body,
            j_body,
        );

        assert_eq!(joint.id(), JointId::new(1));
        assert_eq!(joint.name(), "joint");
        assert_eq!(joint.kind(), JointKind::Spherical);
        assert_eq!(joint.i_body(), i_body);
        assert_eq!(joint.j_body(), j_body);
        assert_eq!(joint.i_marker().name(), "i-marker");
        assert_eq!(joint.j_marker().name(), "j-marker");
    }

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

        let model = Model::new(bodies, joints)?;

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

        let model = Model::new(bodies, joints)?;

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

        let model = Model::new(bodies, joints);

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

        let model = Model::new(bodies, joints);

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

        let model = Model::new(bodies, joints);

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

        let model = Model::new(bodies, joints);

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

        let model = Model::new(bodies, joints);

        assert!(matches!(model, Err(ModelBuildError::MissingGround)));

        Ok(())
    }

    #[test]
    fn rejects_duplicate_joint_id() -> Result<(), ModelBuildError> {
        let joints = Joints::new(vec![
            joint(JointId::new(1), "joint_1", BodyId::new(0), BodyId::new(1)),
            joint(JointId::new(1), "joint_1", BodyId::new(1), BodyId::new(2)),
        ]);

        assert!(matches!(
        joints,
        Err(ModelBuildError::DuplicateJointId(id)) if id == JointId::new(1)
        ));

        Ok(())
    }

    #[test]
    fn rejects_duplicate_body_id() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "body 1"),
            body(BodyId::new(1), "body 1"),
        ]);

        assert!(matches!(
        bodies,
        Err(ModelBuildError::DuplicateBodyId(id)) if id == BodyId::new(1)
        ));

        Ok(())
    }

    #[test]
    fn accepts_ground_only_model() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![body(BodyId::GROUND, "ground")])?;
        let joints = Joints::new(vec![])?;

        assert!(Model::new(bodies, joints).is_ok());

        Ok(())
    }

    #[test]
    fn rejects_non_ground_body_without_joints() -> Result<(), ModelBuildError> {
        let bodies = Bodies::new(vec![
            body(BodyId::GROUND, "ground"),
            body(BodyId::new(1), "free body"),
        ])?;
        let joints = Joints::new(vec![])?;

        let model = Model::new(bodies, joints);

        assert!(matches!(
            model,
            Err(ModelBuildError::FreeBody(id))
                if id == BodyId::new(1)
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

    fn marker(name: &str) -> Marker {
        Marker::new(name, Vector3::zeros(), Rotation3::identity())
    }

    fn joint(id: JointId, name: &str, i_body: BodyId, j_body: BodyId) -> Joint {
        Joint::new(
            id,
            name,
            JointKind::Spherical,
            marker("i"),
            marker("j"),
            i_body,
            j_body,
        )
    }
}
