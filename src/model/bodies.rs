use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::error::Error;
use std::fmt;

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::Hardpoints;

#[derive(Debug, Clone, PartialEq)]
pub struct Bodies {
    pub bodies: BTreeMap<String, Body>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Body {
    pub name: String,
    pub id: BodyId,
    pub side: Side,
    pub pose: Pose,
    pub points: BTreeMap<String, BodyPoint>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodySpec {
    pub name: String,
    pub id: BodyId,
    pub side: Side,
    pub pose: Pose,
    pub point_names: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Pose {
    /// Global position of the body reference point
    pub position: Vector3<f64>,

    /// Body orientation as normalized Euler parameters, stored as a unit quaternion.
    pub orientation: UnitQuaternion<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(pub u32);

#[derive(Debug, Clone, PartialEq)]
pub enum Side {
    Left,
    Right,
    Single,
}

impl Pose {
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

#[derive(Debug, Clone, PartialEq)]
pub struct BodyPoint {
    /// Point position in the body-fixed local frame.
    pub local_position: Vector3<f64>,
}

impl BodyPoint {
    pub fn new(local_position: Vector3<f64>) -> Self {
        Self { local_position }
    }
}

impl Body {
    pub fn position(&self) -> Vector3<f64> {
        self.pose.position
    }

    pub fn global_points(&self) -> BTreeMap<String, Vector3<f64>> {
        self.points
            .iter()
            .map(|(name, point)| {
                (
                    name.clone(),
                    self.pose.local_to_global(point.local_position),
                )
            })
            .collect()
    }
}

impl Bodies {
    pub fn new(bodies: BTreeMap<String, Body>) -> Self {
        Self { bodies }
    }

    pub fn build(specs: &[BodySpec], hardpoints: &Hardpoints) -> Result<Self, BuildBodiesError> {
        let mut bodies = BTreeMap::new();
        let mut used_ids = BTreeSet::new();

        insert_body(&mut bodies, &mut used_ids, build_ground_body(hardpoints))?;

        for spec in specs {
            insert_body(
                &mut bodies,
                &mut used_ids,
                build_body_from_spec(spec, hardpoints)?,
            )?;
        }

        Ok(Self::new(bodies))
    }

    pub fn iter(&self) -> impl Iterator<Item = (&String, &Body)> {
        self.bodies.iter()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuildBodiesError {
    DuplicateBodyId(BodyId),
    MissingHardpoint {
        body_name: String,
        point_name: String,
    },
}

impl fmt::Display for BuildBodiesError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBodyId(id) => write!(f, "duplicate body id: {}", id.0),
            Self::MissingHardpoint {
                body_name,
                point_name,
            } => write!(
                f,
                "body '{body_name}' references missing hardpoint '{point_name}'"
            ),
        }
    }
}

impl Error for BuildBodiesError {}

impl Side {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
            Self::Single => "single",
        }
    }

    pub fn label_short(&self) -> &'static str {
        match self {
            Self::Left => "l",
            Self::Right => "r",
            Self::Single => "s",
        }
    }
}

fn build_ground_body(hardpoints: &Hardpoints) -> Body {
    let mut points = BTreeMap::new();

    for (point_name, point) in hardpoints.iter() {
        // Hardpoints mirrored across the XZ plane and added
        // to the ground body for left and right sides.
        points.insert(
            point_key(BodyId(0), point_name, Side::Left),
            BodyPoint::new(*point),
        );
        points.insert(
            point_key(BodyId(0), point_name, Side::Right),
            BodyPoint::new(mirror_xz(point)),
        );
    }

    // Returning ground body
    Body {
        name: "ground".to_string(),
        id: BodyId(0),
        side: Side::Single,
        pose: Pose::identity(),
        points,
    }
}

fn build_body_from_spec(
    spec: &BodySpec,
    hardpoints: &Hardpoints,
) -> Result<Body, BuildBodiesError> {
    let mut points = BTreeMap::new();

    for point_name in &spec.point_names {
        let point =
            hardpoints
                .get(point_name)
                .ok_or_else(|| BuildBodiesError::MissingHardpoint {
                    body_name: spec.name.clone(),
                    point_name: point_name.clone(),
                })?;

        let global_position = point_for_side(point, &spec.side);
        let local_position = spec.pose.global_to_local(global_position);

        points.insert(
            point_key(spec.id, point_name, spec.side.clone()),
            BodyPoint::new(local_position),
        );
    }

    Ok(Body {
        name: spec.name.clone(),
        id: spec.id,
        side: spec.side.clone(),
        pose: spec.pose.clone(),
        points,
    })
}

fn insert_body(
    bodies: &mut BTreeMap<String, Body>,
    used_ids: &mut BTreeSet<BodyId>,
    body: Body,
) -> Result<(), BuildBodiesError> {
    if !used_ids.insert(body.id) {
        return Err(BuildBodiesError::DuplicateBodyId(body.id));
    }

    bodies.insert(body.name.clone(), body);
    Ok(())
}

fn point_for_side(point: &Vector3<f64>, side: &Side) -> Vector3<f64> {
    match side {
        Side::Left | Side::Single => *point,
        Side::Right => mirror_xz(point),
    }
}

fn point_key(body_id: BodyId, point_name: &str, side: Side) -> String {
    let point_name = point_name.to_lowercase();
    let body_id = body_id.0;

    if body_id == 0 {
        format!("{point_name}0_{}", side.label_short())
    } else {
        format!("{point_name}{}", body_id)
    }
}

fn mirror_xz(point: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(point.x, -point.y, point.z)
}
