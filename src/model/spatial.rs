use nalgebra::{UnitQuaternion, Vector3};

use super::mechanism::Marker;

/// Pose of a body's reference frame in world coordinates.
pub struct BodyPose {
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
}

impl BodyPose {
    pub fn new(position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }

    pub fn marker_orientation(&self, marker: &Marker) -> UnitQuaternion<f64> {
        self.orientation() * marker.orientation()
    }

    pub fn marker_position(&self, marker: &Marker) -> Vector3<f64> {
        self.position() + self.orientation().transform_vector(&marker.position())
    }

    pub fn marker_velocity(&self, marker: &Marker, twist: BodyTwist) -> Vector3<f64> {
        twist.linear_velocity
            + twist
                .angular_velocity
                .cross(&self.orientation().transform_vector(&marker.position()))
    }
}

/// Linear velocity of a body's reference-frame origin and angular velocity
/// of the body, both expressed in world coordinates.
#[derive(Clone, Copy, Debug)]
pub struct BodyTwist {
    linear_velocity: Vector3<f64>,
    angular_velocity: Vector3<f64>,
}

impl BodyTwist {
    /// Creates a body twist from linear and angular velocities expressed
    /// in world coordinates.
    pub fn new(linear_velocity: Vector3<f64>, angular_velocity: Vector3<f64>) -> Self {
        Self {
            linear_velocity,
            angular_velocity,
        }
    }

    /// Returns the velocity of the body's reference-frame origin
    /// in world coordinates.
    pub fn linear_velocity(&self) -> Vector3<f64> {
        self.linear_velocity
    }

    /// Returns the body's angular velocity in world coordinates.
    pub fn angular_velocity(&self) -> Vector3<f64> {
        self.angular_velocity
    }
}
