use crate::model::{BodyId, JointId, Marker};
use crate::problem::{PreparedProblem, tree::TraversalDirection};
use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

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

pub fn solve(problem: &PreparedProblem) -> Result<BodyPoses, SolverError> {
    if !problem.closure_joint_ids().is_empty() {
        return Err(SolverError::ClosedLoopsUnsupported);
    }

    Ok(tree_poses(problem))
}

pub fn tree_poses(problem: &PreparedProblem) -> BodyPoses {
    let mut poses = BTreeMap::new();

    poses.insert(
        BodyId::GROUND,
        BodyPose::new(Vector3::zeros(), UnitQuaternion::identity()),
    );

    for edge in problem.tree_edges() {
        let joint = problem
            .model()
            .joints()
            .get(edge.joint_id())
            .expect("PreparedProblem contains missing joint");

        let parent = poses
            .get(&edge.parent_body_id())
            .expect("PreparedProblem contains unsolved parent");

        let (parent_marker, child_marker) = match edge.direction() {
            TraversalDirection::IToJ => (joint.i_marker(), joint.j_marker()),
            TraversalDirection::JToI => (joint.j_marker(), joint.i_marker()),
        };
        let child = spherical_child_pose(
            parent,
            parent_marker,
            child_marker,
            edge.traversal_relative_orientation(Vector3::zeros()),
        );

        poses.insert(edge.child_body_id(), child);
    }

    BodyPoses { poses }
}

pub fn closure_position_residuals(
    problem: &PreparedProblem,
    poses: &BodyPoses,
) -> Vec<Vector3<f64>> {
    problem
        .closure_joint_ids()
        .iter()
        .map(|joint_id| {
            let joint = problem
                .model()
                .joints()
                .get(*joint_id)
                .expect("PreparedProblem contains missing closure joint");

            let i_pose = poses
                .get(joint.i_marker().body_id())
                .expect("closure joint i body has no pose");

            let j_pose = poses
                .get(joint.j_marker().body_id())
                .expect("closure joint j body has no pose");

            i_pose.marker_position(joint.i_marker()) - j_pose.marker_position(joint.j_marker())
        })
        .collect()
}

pub fn closure_orientation_residuals(problem: &PreparedProblem, poses: &BodyPoses) -> Vec<f64> {
    problem
        .closure_joint_ids()
        .iter()
        .flat_map(|joint_id| {
            let joint = problem
                .model()
                .joints()
                .get(*joint_id)
                .expect("PreparedProblem contains missing closure joint");
            let coordinate = problem
                .joint_coordinate(*joint_id)
                .expect("PreparedProblem contains missing joint coordinate");
            let i_pose = poses
                .get(joint.i_marker().body_id())
                .expect("closure joint i body has no pose");
            let j_pose = poses
                .get(joint.j_marker().body_id())
                .expect("closure joint j body has no pose");
            let actual = i_pose.marker_orientation(joint.i_marker()).inverse()
                * j_pose.marker_orientation(joint.j_marker());
            let actual_displacement =
                (actual * coordinate.reference_orientation().inverse()).scaled_axis();
            let requested = coordinate.displacement().rotation();

            [requested.x, requested.y, requested.z]
                .into_iter()
                .enumerate()
                .filter_map(move |(index, requested)| {
                    requested.map(|requested| actual_displacement[index] - requested)
                })
        })
        .collect()
}

pub struct BodyPose {
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyTwist {
    linear_velocity: Vector3<f64>,
    angular_velocity: Vector3<f64>,
}

impl BodyTwist {
    pub fn new(linear_velocity: Vector3<f64>, angular_velocity: Vector3<f64>) -> Self {
        Self {
            linear_velocity,
            angular_velocity,
        }
    }
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

    pub fn marker_velocity(&self, marker: &Marker, twist: BodyTwist) -> Vector3<f64> {
        twist.linear_velocity
            + twist
                .angular_velocity
                .cross(&self.orientation().transform_vector(&marker.position()))
    }
}

pub fn closure_position_residual_rates(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<Vector3<f64>>, ResidualError> {
    let mut rates = Vec::new();

    for joint_id in problem.closure_joint_ids() {
        let joint = problem
            .model()
            .joints()
            .get(*joint_id)
            .expect("PreparedProblem contains missing closure joint");
        let i_pose = poses
            .get(joint.i_marker().body_id())
            .expect("closure joint i body has no pose");
        let j_pose = poses
            .get(joint.j_marker().body_id())
            .expect("closure joint j body has no pose");
        let i_twist =
            *twists
                .get(&joint.i_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.i_marker().body_id(),
                })?;
        let j_twist =
            *twists
                .get(&joint.j_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.j_marker().body_id(),
                })?;

        rates.push(
            i_pose.marker_velocity(joint.i_marker(), i_twist)
                - j_pose.marker_velocity(joint.j_marker(), j_twist),
        );
    }

    Ok(rates)
}

pub fn closure_orientation_residual_rates(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<f64>, ResidualError> {
    let mut rates = Vec::new();

    for joint_id in problem.closure_joint_ids() {
        let joint = problem
            .model()
            .joints()
            .get(*joint_id)
            .expect("PreparedProblem contains missing closure joint");
        let coordinate = problem
            .joint_coordinate(*joint_id)
            .expect("PreparedProblem contains missing joint coordinate");
        let i_pose = poses
            .get(joint.i_marker().body_id())
            .expect("closure joint i body has no pose");
        let j_pose = poses
            .get(joint.j_marker().body_id())
            .expect("closure joint j body has no pose");
        let i_twist =
            *twists
                .get(&joint.i_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.i_marker().body_id(),
                })?;
        let j_twist =
            *twists
                .get(&joint.j_marker().body_id())
                .ok_or(ResidualError::MissingBodyTwist {
                    joint_id: *joint_id,
                    body_id: joint.j_marker().body_id(),
                })?;
        let actual = i_pose.marker_orientation(joint.i_marker()).inverse()
            * j_pose.marker_orientation(joint.j_marker());
        let actual_displacement =
            (actual * coordinate.reference_orientation().inverse()).scaled_axis();
        let relative_angular_velocity = i_pose
            .marker_orientation(joint.i_marker())
            .inverse_transform_vector(&(j_twist.angular_velocity - i_twist.angular_velocity));
        let actual_displacement_rate = coordinate.displacement_rate_from_relative_angular_velocity(
            actual_displacement,
            relative_angular_velocity,
        );
        let requested = coordinate.displacement().rotation();

        for (index, requested) in [requested.x, requested.y, requested.z]
            .into_iter()
            .enumerate()
        {
            if requested.is_some() {
                rates.push(actual_displacement_rate[index]);
            }
        }
    }

    Ok(rates)
}

pub fn closure_residuals(problem: &PreparedProblem, poses: &BodyPoses) -> Vec<f64> {
    let position_rows = closure_position_residuals(problem, poses)
        .into_iter()
        .flat_map(|residual| [residual.x, residual.y, residual.z]);

    position_rows
        .chain(closure_orientation_residuals(problem, poses))
        .collect()
}

pub fn closure_residual_rates(
    problem: &PreparedProblem,
    poses: &BodyPoses,
    twists: &BTreeMap<BodyId, BodyTwist>,
) -> Result<Vec<f64>, ResidualError> {
    let position_rows = closure_position_residual_rates(problem, poses, twists)?
        .into_iter()
        .flat_map(|rate| [rate.x, rate.y, rate.z]);

    Ok(position_rows
        .chain(closure_orientation_residual_rates(problem, poses, twists)?)
        .collect())
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

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum SolverError {
    #[error("closed-loop mechanisms are not supported by the solver yet")]
    ClosedLoopsUnsupported,
}

#[derive(Debug, Error)]
pub enum ResidualError {
    #[error("closure joint `{joint_id:?}` body `{body_id:?}` has no twist")]
    MissingBodyTwist { joint_id: JointId, body_id: BodyId },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::yaml::parse_yaml_str;
    use crate::model::BodyId;
    use crate::problem::prepare;

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

    #[test]
    fn evaluates_closed_loop_position_residuals_from_tree_poses() {
        let input = parse_yaml_str(include_str!(
            "../../examples/spherical_one_body_closed_loop_motion.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let residuals = closure_position_residuals(&problem, &poses);

        assert_eq!(residuals.len(), 1);
        assert!(residuals[0].norm() < TOLERANCE);
    }

    #[test]
    fn evaluates_prescribed_closed_loop_orientation_residuals() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../examples/spherical_one_body_closed_loop_motion.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let residuals = closure_orientation_residuals(&problem, &poses);

        assert_eq!(residuals.len(), 1);
        assert!(residuals[0].abs() < TOLERANCE);
    }

    #[test]
    fn evaluates_closed_loop_position_residual_rates_from_body_twists() {
        let input = parse_yaml_str(include_str!(
            "../../examples/spherical_one_body_closed_loop_motion.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let twists = BTreeMap::from([
            (
                BodyId::GROUND,
                BodyTwist::new(Vector3::zeros(), Vector3::zeros()),
            ),
            (
                BodyId::new(1),
                BodyTwist::new(Vector3::zeros(), Vector3::x()),
            ),
        ]);
        let rates = closure_position_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(rates.len(), 1);
        assert!(rates[0].norm() > 1.0);
    }

    #[test]
    fn rejects_missing_body_twist_for_closure_rates() {
        let input = parse_yaml_str(include_str!(
            "../../examples/spherical_one_body_closed_loop_motion.yaml"
        ))
        .unwrap()
        .into_input()
        .unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);

        assert!(matches!(
            closure_position_residual_rates(&problem, &poses, &BTreeMap::new()),
            Err(ResidualError::MissingBodyTwist { .. })
        ));
    }

    #[test]
    fn evaluates_prescribed_closed_loop_orientation_residual_rates() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../examples/spherical_one_body_closed_loop_motion.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let twists = BTreeMap::from([
            (
                BodyId::GROUND,
                BodyTwist::new(Vector3::zeros(), Vector3::zeros()),
            ),
            (
                BodyId::new(1),
                BodyTwist::new(Vector3::zeros(), Vector3::z()),
            ),
        ]);
        let rates = closure_orientation_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(rates.len(), 1);
        assert!((rates[0] + 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn orders_position_rows_before_orientation_rows() {
        let yaml = format!(
            "{}\n  RotateJ2:\n    kind: joint-coordinates\n    joint_id: 2\n    displacement:\n      rot_z: -90\n",
            include_str!("../../examples/spherical_one_body_closed_loop_motion.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = tree_poses(&problem);
        let twists = BTreeMap::from([
            (
                BodyId::GROUND,
                BodyTwist::new(Vector3::zeros(), Vector3::zeros()),
            ),
            (
                BodyId::new(1),
                BodyTwist::new(Vector3::zeros(), Vector3::z()),
            ),
        ]);
        let residuals = closure_residuals(&problem, &poses);
        let rates = closure_residual_rates(&problem, &poses, &twists).unwrap();

        assert_eq!(residuals.len(), 4);
        assert!(residuals.iter().all(|residual| residual.abs() < TOLERANCE));
        assert_eq!(rates.len(), 4);
        assert!(rates[..3].iter().all(|rate| rate.abs() < TOLERANCE));
        assert!((rates[3] + 1.0).abs() < TOLERANCE);
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
