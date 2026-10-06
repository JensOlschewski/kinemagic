use std::collections::HashSet;

use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

use super::Point;

/// Collection of bodies with unique identifiers.
pub struct Bodies {
    bodies: Vec<Body>,
}

impl Bodies {
    /// Creates body collection.
    ///
    /// # Errors
    ///
    /// Returns [`ModelBuildError::DuplicateBodyId`] when multiple bodies share an ID.
    pub fn new(bodies: Vec<Body>) -> Result<Self, BodyError> {
        let mut ids = HashSet::new();

        for body in &bodies {
            if !ids.insert(body.id()) {
                return Err(BodyError::DuplicateBodyId(body.id()));
            }
        }

        Ok(Self { bodies })
    }

    /// Iterates over bodies in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = &Body> {
        self.bodies.iter()
    }

    /// Returns body with given ID, if present.
    pub fn get(&self, id: BodyId) -> Option<&Body> {
        self.iter().find(|body| body.id() == id)
    }

    /// Returns `true` when collection contains given body ID.
    pub fn contains(&self, id: BodyId) -> bool {
        self.get(id).is_some()
    }
}

/// Rigid body with pose and named local points.
pub struct Body {
    id: BodyId,
    name: String,
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
    points: Vec<Point>,
}

impl Body {
    /// Creates rigid body.
    ///
    /// `position` and `orientation` define body pose in global coordinates.
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

    /// Returns body-origin position in global coordinates.
    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    /// Returns body orientation relative to global coordinates.
    pub fn orientation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }

    /// Returns points fixed in body-local coordinates.
    pub fn points(&self) -> &[Point] {
        &self.points
    }
}

/// Stable identifier for a body in a mechanism.
///
/// Value `0` is reserved for [`BodyId::GROUND`].
/// Identifier of ground body.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
pub struct BodyId(u32);

impl BodyId {
    pub const GROUND: Self = Self(0);

    /// Creates identifier from numeric value.
    ///
    /// Value `0` denotes ground.
    pub fn new(value: u32) -> Self {
        Self(value)
    }
}

impl std::fmt::Display for BodyId {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}", self.0)
    }
}

/// Errors returned when constructing a body.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum BodyError {
    #[error("duplicate body ID `{0:?}`")]
    DuplicateBodyId(BodyId),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::mechanism::Marker;

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
    fn rejects_duplicate_body_id() -> Result<(), BodyError> {
        let bodies = Bodies::new(vec![
            create_body(BodyId::GROUND, "ground"),
            create_body(BodyId::new(1), "body 1"),
            create_body(BodyId::new(1), "body 1"),
        ]);

        assert!(matches!(
        bodies,
        Err(BodyError::DuplicateBodyId(id)) if id == BodyId::new(1)
        ));

        Ok(())
    }

    fn create_body(id: BodyId, name: &str) -> Body {
        Body::new(
            id,
            name,
            Vector3::zeros(),
            UnitQuaternion::identity(),
            vec![],
        )
    }
}
