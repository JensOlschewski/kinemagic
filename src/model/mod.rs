use std::collections::HashSet;

use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

pub struct Input {
    model: Model,
    motions: Vec<Motion>,
    solver: SolverSettings,
}

impl Input {
    pub fn new(model: Model, motions: Vec<Motion>) -> Self {
        Self::with_solver(model, motions, SolverSettings::defaults())
    }

    pub fn with_solver(model: Model, motions: Vec<Motion>, solver: SolverSettings) -> Self {
        Self {
            model,
            motions,
            solver,
        }
    }

    pub fn model(&self) -> &Model {
        &self.model
    }

    pub fn motions(&self) -> &[Motion] {
        &self.motions
    }

    pub fn solver(&self) -> SolverSettings {
        self.solver
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolverSettings {
    start_time: f64,
    end_time: f64,
    step_size: f64,
}

impl SolverSettings {
    pub const fn defaults() -> Self {
        Self {
            start_time: 0.0,
            end_time: 1.0,
            step_size: 1.0,
        }
    }

    pub fn new(start_time: f64, end_time: f64, step_size: f64) -> Self {
        Self {
            start_time,
            end_time,
            step_size,
        }
    }

    pub fn start_time(&self) -> f64 {
        self.start_time
    }
    pub fn end_time(&self) -> f64 {
        self.end_time
    }
    pub fn step_size(&self) -> f64 {
        self.step_size
    }

    pub fn times(&self) -> impl Iterator<Item = f64> {
        let count = ((self.end_time - self.start_time) / self.step_size).round() as usize;
        (0..=count).map(move |index| self.start_time + index as f64 * self.step_size)
    }
}

pub struct Model {
    bodies: Bodies,
    joints: Joints,
}

impl Model {
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

        // BodyId(0) is reserved for ground
        if !bodies.contains(BodyId::GROUND) {
            return Err(ModelBuildError::MissingGround);
        }

        // Check if free body exists
        let mut reachable = HashSet::from([BodyId::GROUND]);

        loop {
            let before = reachable.len();

            for joint in joints.iter() {
                if reachable.contains(&joint.i_marker.body_id) {
                    reachable.insert(joint.j_marker.body_id);
                }
                if reachable.contains(&joint.j_marker.body_id) {
                    reachable.insert(joint.i_marker.body_id);
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

    pub fn iter(&self) -> impl Iterator<Item = &Body> {
        self.bodies.iter()
    }

    pub fn get(&self, id: BodyId) -> Option<&Body> {
        self.iter().find(|body| body.id() == id)
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

    pub fn get(&self, id: JointId) -> Option<&Joint> {
        self.iter().find(|joint| joint.id() == id)
    }

    pub fn contains(&self, id: JointId) -> bool {
        self.get(id).is_some()
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
    role: JointRole,
    i_marker: Marker,
    j_marker: Marker,
}

impl Joint {
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

pub struct Motion {
    name: String,
    kind: MotionKind,
    joint_id: JointId,
    joint_displacement: JointDisplacement,
}

impl Motion {
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

pub struct Marker {
    name: String,
    body_id: BodyId,
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
}

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JointRole {
    Auto,
    Primary,
    Secondary,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum MotionKind {
    JointCoordinates,
}

#[derive(Debug, Clone, Copy)]
pub struct JointDisplacement {
    // Add more joints later here
    rotation: Vector3<Option<f64>>,
    rotation_rate: Vector3<Option<f64>>,
}

impl JointDisplacement {
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

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
pub struct BodyId(u32);

impl BodyId {
    // BodyId(0) is reserved for ground.
    pub const GROUND: Self = Self(0);

    pub fn new(value: u32) -> Self {
        Self(value)
    }
}

impl std::fmt::Display for BodyId {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}", self.0)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
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
    #[error("joint `{joint_id:?}` connects body `{body_id:?}` to itself")]
    SelfConnectingJoint { joint_id: JointId, body_id: BodyId },
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
        let orientation = UnitQuaternion::identity();
        let body_id = BodyId::new(0);
        let marker = Marker::new("marker", body_id, position, orientation);

        assert_eq!(marker.name(), "marker");
        assert_eq!(marker.body_id(), body_id);
        assert_eq!(marker.position(), position);
        assert_eq!(marker.orientation(), orientation);
    }

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

        let model = Model::new(bodies, joints);

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
