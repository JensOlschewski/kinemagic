use crate::model::{BodyId, Marker};
use crate::problem::PreparedProblem;
use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};

pub struct BodyPoses {
    poses: BTreeMap<BodyId, BodyPose>,
}

impl BodyPoses {
    pub fn get(&self, body_id: BodyId) -> Option<&BodyPose> {
        self.poses.get(&body_id)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&BodyId, &BodyPose)> {
        self.poses.iter()
    }
}

pub fn solve(problem: &PreparedProblem) -> BodyPoses {
    let mut poses = BTreeMap::new();

    poses.insert(
        BodyId::GROUND,
        BodyPose::new(Vector3::zeros(), UnitQuaternion::identity()),
    );

    for step in problem.steps() {
        let joint = problem
            .model()
            .joints()
            .get(step.joint_id())
            .expect("PreparedProblem contains missing joint");

        let parent = poses
            .get(&step.parent_id())
            .expect("PreparedProblem contains unsolved parent");

        let child = spherical_child_pose(
            parent,
            joint.i_marker(),
            joint.j_marker(),
            step.relative_orientation(),
        );

        poses.insert(step.child_id(), child);
    }

    BodyPoses { poses }
}

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

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn marker_orientation(&self, marker: &Marker) -> UnitQuaternion<f64> {
        self.orientation() * marker.orientation()
    }

    pub fn marker_position(&self, marker: &Marker) -> Vector3<f64> {
        self.position() + self.orientation().transform_vector(&marker.position())
    }
}

pub fn spherical_child_pose(
    parent: &BodyPose,
    i_marker: &Marker,
    j_marker: &Marker,
    relative_orientation: UnitQuaternion<f64>,
) -> BodyPose {
    // R_child = R_parent A_i Q A_j⁻¹
    let child_orientation = parent.marker_orientation(i_marker)
        * relative_orientation
        * j_marker.orientation().inverse();

    // r_child = p_i_world − R_child s_j
    let child_position =
        parent.marker_position(i_marker) - child_orientation.transform_vector(&j_marker.position());

    BodyPose::new(child_position, child_orientation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::BodyId;

    const TOLERANCE: f64 = 1.0e-12;

    #[test]
    fn identity_markers_preserve_parent_pose() {
        let parent = parent_pose();
        let i_marker = marker(BodyId::GROUND, Vector3::zeros(), UnitQuaternion::identity());
        let j_marker = marker(BodyId::new(1), Vector3::zeros(), UnitQuaternion::identity());

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, UnitQuaternion::identity());

        assert_position_close(child.position(), parent.position());
        assert_orientation_close(child.orientation(), parent.orientation());
    }

    #[test]
    fn computes_child_pose_from_relative_orientation() {
        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let parent = parent_pose();
        let i_marker = marker(BodyId::GROUND, Vector3::zeros(), UnitQuaternion::identity());
        let j_marker = marker(BodyId::new(1), Vector3::zeros(), UnitQuaternion::identity());

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        assert_position_close(child.position(), parent.position());
        assert_orientation_close(
            child.orientation(),
            parent.orientation() * relative_orientation,
        );
    }

    #[test]
    fn combines_non_aligned_marker_frames_with_relative_orientation() {
        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let parent = parent_pose();

        let i_marker = marker(
            BodyId::GROUND,
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.2, -0.1, 0.3),
        );
        let j_marker = marker(
            BodyId::new(1),
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(-0.4, 0.5, -0.2),
        );

        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        let expected = parent.marker_orientation(&i_marker) * relative_orientation;
        let actual = child.marker_orientation(&j_marker);

        assert_position_close(child.position(), parent.position());
        assert_orientation_close(actual, expected);
    }

    #[test]
    fn handles_nonzero_marker_offsets() {
        let parent = BodyPose::new(Vector3::new(1.0, 2.0, 3.0), UnitQuaternion::identity());
        let i_marker = marker(
            BodyId::GROUND,
            Vector3::new(2.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let j_marker = marker(
            BodyId::new(1),
            Vector3::new(0.0, 1.0, 0.0),
            UnitQuaternion::identity(),
        );

        let relative_orientation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
        let child = spherical_child_pose(&parent, &i_marker, &j_marker, relative_orientation);

        let i_world = parent.marker_position(&i_marker);
        let j_world = child.marker_position(&j_marker);

        assert_position_close(i_world, j_world);
        assert_orientation_close(child.orientation(), relative_orientation);
    }

    fn parent_pose() -> BodyPose {
        BodyPose::new(
            Vector3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        )
    }

    fn marker(body_id: BodyId, position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Marker {
        Marker::new("marker", body_id, position, orientation)
    }

    fn assert_position_close(actual: Vector3<f64>, expected: Vector3<f64>) {
        assert!((actual - expected).norm() < TOLERANCE);
    }

    fn assert_orientation_close(actual: UnitQuaternion<f64>, expected: UnitQuaternion<f64>) {
        assert!(actual.angle_to(&expected) < TOLERANCE);
    }
}
