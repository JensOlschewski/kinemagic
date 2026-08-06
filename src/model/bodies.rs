use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::error::Error;
use std::fmt;

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::{Hardpoints, Side};

pub struct Bodies {
    bodies: BTreeMap<BodyId, Body>,
}

pub struct Body {
    name: String,
    id: BodyId,
    side: Side,
    pose: BodyPose,
    points: BTreeMap<String, BodyPoint>,
}

pub struct BodySpec {
    pub name: String,
    pub id: BodyId,
    pub side: Side,
    pub pose: BodyPose,
    pub point_names: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
/// u16 2^16 bodies are possible, which is more than
/// enough for any practical application.
pub struct BodyId(pub u16);

impl BodyId {
    pub const GROUND: Self = Self(0);
}

#[derive(Clone)]
pub struct BodyPose {
    /// Global position of the body reference point
    pub position: Vector3<f64>,
    /// Body orientation as normalized Euler parameters, stored as a unit quaternion.
    pub orientation: UnitQuaternion<f64>,
}

pub struct BodyPoint {
    /// Point position in the body-fixed local frame.
    pub local_position: Vector3<f64>,
}

impl Bodies {
    fn new(bodies: BTreeMap<BodyId, Body>) -> Self {
        Self { bodies }
    }

    pub fn build(specs: &[BodySpec], hardpoints: &Hardpoints) -> Result<Self, BodiesError> {
        let mut bodies = BTreeMap::new();
        let mut used_names = BTreeSet::new();

        insert_body(&mut bodies, &mut used_names, build_ground_body(hardpoints))?;

        for spec in specs {
            insert_body(
                &mut bodies,
                &mut used_names,
                build_body_from_spec(spec, hardpoints)?,
            )?;
        }

        Ok(Self::new(bodies))
    }

    pub fn iter(&self) -> impl Iterator<Item = &Body> {
        self.bodies.values()
    }

    pub fn get_by_id(&self, body_id: BodyId) -> Option<&Body> {
        self.bodies.get(&body_id)
    }

    pub fn global_point(
        &self,
        body_id: BodyId,
        point_name: &str,
        side: Side,
    ) -> Result<Vector3<f64>, BodiesError> {
        let body = self
            .get_by_id(body_id)
            .ok_or(BodiesError::MissingBodyId(body_id))?;

        let point = body
            .global_point(point_name)
            .ok_or_else(|| BodiesError::MissingPoint {
                id: body_id,
                point_name: point_name.to_string(),
            })?;

        let point = if body_id == BodyId::GROUND {
            point_for_side(&point, &side)
        } else {
            point
        };

        Ok(point)
    }
}

impl Body {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn id(&self) -> BodyId {
        self.id
    }

    pub fn side(&self) -> Side {
        self.side
    }

    pub fn pose(&self) -> &BodyPose {
        &self.pose
    }

    pub fn position(&self) -> Vector3<f64> {
        self.pose.position
    }

    pub fn global_point(&self, point_name: &str) -> Option<Vector3<f64>> {
        self.points
            .get(point_name)
            .map(|point| self.pose.local_to_global(point.local_position))
    }

    pub fn global_points(&self) -> BTreeMap<String, Vector3<f64>> {
        self.global_points_at(&self.pose)
    }

    pub fn global_points_at(&self, pose: &BodyPose) -> BTreeMap<String, Vector3<f64>> {
        self.points
            .iter()
            .map(|(name, point)| (name.clone(), pose.local_to_global(point.local_position)))
            .collect()
    }
}

impl BodyPose {
    pub fn new(position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn identity() -> Self {
        Self::new(Vector3::zeros(), UnitQuaternion::identity())
    }

    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        self.orientation.to_rotation_matrix().into_inner()
    }

    pub fn local_to_global(&self, local_position: Vector3<f64>) -> Vector3<f64> {
        self.position + self.rotation_matrix() * local_position
    }

    pub fn global_to_local(&self, global_position: Vector3<f64>) -> Vector3<f64> {
        self.rotation_matrix().transpose() * (global_position - self.position)
    }
}

impl BodyPoint {
    pub fn new(local_position: Vector3<f64>) -> Self {
        Self { local_position }
    }
}

#[derive(Debug)]
pub enum BodiesError {
    DuplicateBodyName(String),
    DuplicateBodyId(BodyId),
    MissingBodyId(BodyId),
    MissingHardpoint { id: BodyId, point_name: String },
    MissingPoint { id: BodyId, point_name: String },
}

impl fmt::Display for BodiesError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBodyName(name) => write!(f, "duplicate body name: {}", name),
            Self::DuplicateBodyId(id) => write!(f, "duplicate body id: {}", id.0),
            Self::MissingBodyId(id) => write!(f, "missing body id: {}", id.0),
            Self::MissingHardpoint { id, point_name } => write!(
                f,
                "body {} references missing hardpoint '{point_name}'",
                id.0
            ),
            Self::MissingPoint { id, point_name } => {
                write!(f, "body '{}' does not contain point '{point_name}'", id.0,)
            }
        }
    }
}

impl Error for BodiesError {}

fn build_ground_body(hardpoints: &Hardpoints) -> Body {
    let points = hardpoints
        .iter()
        .map(|(name, point)| (name.clone(), BodyPoint::new(*point)))
        .collect();

    // Returning ground body
    Body {
        name: "ground".to_string(),
        id: BodyId::GROUND,
        side: Side::Single,
        pose: BodyPose::identity(),
        points,
    }
}

fn build_body_from_spec(spec: &BodySpec, hardpoints: &Hardpoints) -> Result<Body, BodiesError> {
    let mut points = BTreeMap::new();

    for name in &spec.point_names {
        let point = hardpoints
            .get(name)
            .ok_or_else(|| BodiesError::MissingHardpoint {
                id: spec.id,
                point_name: name.clone(),
            })?;

        let global_position = point_for_side(point, &spec.side);
        let local_position = spec.pose.global_to_local(global_position);

        points.insert(name.clone(), BodyPoint::new(local_position));
    }

    Ok(Body {
        name: spec.name.clone(),
        id: spec.id,
        side: spec.side,
        pose: spec.pose.clone(),
        points,
    })
}

fn insert_body(
    bodies: &mut BTreeMap<BodyId, Body>,
    used_names: &mut BTreeSet<String>,
    body: Body,
) -> Result<(), BodiesError> {
    if bodies.contains_key(&body.id) {
        return Err(BodiesError::DuplicateBodyId(body.id));
    }

    if !used_names.insert(body.name.clone()) {
        return Err(BodiesError::DuplicateBodyName(body.name.clone()));
    }

    bodies.insert(body.id, body);
    Ok(())
}

fn point_for_side(point: &Vector3<f64>, side: &Side) -> Vector3<f64> {
    match side {
        Side::Left | Side::Single => *point,
        Side::Right => mirror_xz(point),
    }
}

fn mirror_xz(point: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(point.x, -point.y, point.z)
}
