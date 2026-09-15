use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector2, Vector3};
use thiserror::Error;

use crate::model::{BodyId, JointId, Model};
use crate::solve::BodyPoses;

mod terminal;

pub use terminal::{LiveViewEvent, ViewFrame, ViewerError, run_live_viewer, run_viewer};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Projection {
    Xy,
    Xz,
    Yz,
    Isometric,
}

impl Projection {
    pub fn project(self, point: Vector3<f64>) -> Vector2<f64> {
        match self {
            Self::Xy => Vector2::new(point.x, point.y),
            Self::Xz => Vector2::new(point.x, point.z),
            Self::Yz => Vector2::new(point.y, point.z),
            Self::Isometric => {
                let inverse_sqrt_two = 1.0 / 2.0_f64.sqrt();
                let inverse_sqrt_six = 1.0 / 6.0_f64.sqrt();
                Vector2::new(
                    (point.x - point.y) * inverse_sqrt_two,
                    (-point.x - point.y + 2.0 * point.z) * inverse_sqrt_six,
                )
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Scene {
    bodies: Vec<SceneBody>,
    joints: Vec<SceneJoint>,
    ground: SceneGround,
}

impl Scene {
    pub fn from_reference(model: &Model) -> Self {
        let world_poses = model
            .bodies()
            .iter()
            .map(|body| {
                (
                    body.id(),
                    WorldPose::new(body.position(), body.orientation()),
                )
            })
            .collect();

        Self::from_world_poses(model, &world_poses)
    }

    pub fn from_body_poses(model: &Model, body_poses: &BodyPoses) -> Result<Self, SceneBuildError> {
        let world_poses = model
            .bodies()
            .iter()
            .map(|body| {
                let body_id = body.id();
                let pose = body_poses
                    .get(body_id)
                    .ok_or(SceneBuildError::MissingBodyPose { body_id })?;

                Ok((body_id, WorldPose::new(pose.position(), pose.orientation())))
            })
            .collect::<Result<_, _>>()?;

        Ok(Self::from_world_poses(model, &world_poses))
    }

    pub fn bodies(&self) -> &[SceneBody] {
        &self.bodies
    }

    pub fn joints(&self) -> &[SceneJoint] {
        &self.joints
    }

    pub fn ground(&self) -> &SceneGround {
        &self.ground
    }

    pub fn projected_bounds(&self, projection: Projection) -> ProjectedBounds {
        ProjectedBounds::for_scenes(std::slice::from_ref(self), projection)
            .expect("a scene always contains ground")
    }

    fn from_world_poses(model: &Model, world_poses: &BTreeMap<BodyId, WorldPose>) -> Self {
        let bodies = model
            .bodies()
            .iter()
            .filter(|body| body.id() != BodyId::GROUND)
            .map(|body| {
                let pose = world_poses[&body.id()];
                let points = body
                    .points()
                    .iter()
                    .map(|point| ScenePoint {
                        label: point.name().to_owned(),
                        position: pose.transform_point(point.position()),
                    })
                    .collect::<Vec<_>>();
                let segments = body_segments(pose.position, &points);

                SceneBody {
                    body_id: body.id(),
                    label: body.name().to_owned(),
                    origin: pose.position,
                    points,
                    segments,
                }
            })
            .collect();

        let ground_body = model
            .bodies()
            .get(BodyId::GROUND)
            .expect("model guarantees a ground body");
        let ground_pose = world_poses[&BodyId::GROUND];
        let ground_points = ground_body
            .points()
            .iter()
            .map(|point| ScenePoint {
                label: point.name().to_owned(),
                position: ground_pose.transform_point(point.position()),
            })
            .collect::<Vec<_>>();
        let ground_segments = body_segments(ground_pose.position, &ground_points);
        let mut ground_markers = Vec::new();
        let joints = model
            .joints()
            .iter()
            .map(|joint| {
                let i_marker = joint.i_marker();
                let j_marker = joint.j_marker();
                let i_position =
                    world_poses[&i_marker.body_id()].transform_point(i_marker.position());
                let j_position =
                    world_poses[&j_marker.body_id()].transform_point(j_marker.position());

                if i_marker.body_id() == BodyId::GROUND {
                    ground_markers.push(i_position);
                }
                if j_marker.body_id() == BodyId::GROUND {
                    ground_markers.push(j_position);
                }

                SceneJoint {
                    joint_id: joint.id(),
                    label: joint.name().to_owned(),
                    i_position,
                    j_position,
                }
            })
            .collect();

        Self {
            bodies,
            joints,
            ground: SceneGround {
                label: ground_body.name().to_owned(),
                origin: ground_pose.position,
                points: ground_points,
                segments: ground_segments,
                markers: ground_markers,
            },
        }
    }

    fn visit_positions(&self, mut visit: impl FnMut(Vector3<f64>)) {
        visit(self.ground.origin);
        for point in &self.ground.points {
            visit(point.position);
        }
        for marker in &self.ground.markers {
            visit(*marker);
        }

        for body in &self.bodies {
            visit(body.origin);
            for point in &body.points {
                visit(point.position);
            }
        }

        for joint in &self.joints {
            visit(joint.i_position);
            visit(joint.j_position);
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SceneBody {
    body_id: BodyId,
    label: String,
    origin: Vector3<f64>,
    points: Vec<ScenePoint>,
    segments: Vec<SceneSegment>,
}

impl SceneBody {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn origin(&self) -> Vector3<f64> {
        self.origin
    }

    pub fn points(&self) -> &[ScenePoint] {
        &self.points
    }

    pub fn segments(&self) -> &[SceneSegment] {
        &self.segments
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScenePoint {
    label: String,
    position: Vector3<f64>,
}

impl ScenePoint {
    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SceneSegment {
    start: Vector3<f64>,
    end: Vector3<f64>,
}

impl SceneSegment {
    pub fn start(&self) -> Vector3<f64> {
        self.start
    }

    pub fn end(&self) -> Vector3<f64> {
        self.end
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SceneJoint {
    joint_id: JointId,
    label: String,
    i_position: Vector3<f64>,
    j_position: Vector3<f64>,
}

impl SceneJoint {
    pub fn joint_id(&self) -> JointId {
        self.joint_id
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn i_position(&self) -> Vector3<f64> {
        self.i_position
    }

    pub fn j_position(&self) -> Vector3<f64> {
        self.j_position
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SceneGround {
    label: String,
    origin: Vector3<f64>,
    points: Vec<ScenePoint>,
    segments: Vec<SceneSegment>,
    markers: Vec<Vector3<f64>>,
}

impl SceneGround {
    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn origin(&self) -> Vector3<f64> {
        self.origin
    }

    pub fn points(&self) -> &[ScenePoint] {
        &self.points
    }

    pub fn segments(&self) -> &[SceneSegment] {
        &self.segments
    }

    pub fn markers(&self) -> &[Vector3<f64>] {
        &self.markers
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProjectedBounds {
    min: Vector2<f64>,
    max: Vector2<f64>,
}

impl ProjectedBounds {
    pub fn for_scenes<'a>(
        scenes: impl IntoIterator<Item = &'a Scene>,
        projection: Projection,
    ) -> Option<Self> {
        let mut bounds = BoundsBuilder::default();
        for scene in scenes {
            scene.visit_positions(|position| bounds.include(projection.project(position)));
        }
        bounds.finish()
    }

    pub fn min(&self) -> Vector2<f64> {
        self.min
    }

    pub fn max(&self) -> Vector2<f64> {
        self.max
    }

    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }

    pub fn padded(self, fraction: f64) -> Self {
        let fraction = if fraction.is_finite() {
            fraction.max(0.0)
        } else {
            0.0
        };
        let padding = Vector2::new(self.width() * fraction, self.height() * fraction);

        Self {
            min: self.min - padding,
            max: self.max + padding,
        }
    }

    pub fn fit_aspect(self, viewport_width: u16, viewport_height: u16) -> Self {
        if viewport_width == 0 || viewport_height == 0 {
            return self;
        }

        let viewport_aspect = f64::from(viewport_width) / f64::from(viewport_height);
        let bounds_aspect = self.width() / self.height();
        let center = (self.min + self.max) / 2.0;

        if bounds_aspect > viewport_aspect {
            let half_height = self.width() / viewport_aspect / 2.0;
            Self {
                min: Vector2::new(self.min.x, center.y - half_height),
                max: Vector2::new(self.max.x, center.y + half_height),
            }
        } else {
            let half_width = self.height() * viewport_aspect / 2.0;
            Self {
                min: Vector2::new(center.x - half_width, self.min.y),
                max: Vector2::new(center.x + half_width, self.max.y),
            }
        }
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum SceneBuildError {
    #[error("solved poses do not contain body `{body_id}`")]
    MissingBodyPose { body_id: BodyId },
}

#[derive(Clone, Copy)]
struct WorldPose {
    position: Vector3<f64>,
    orientation: UnitQuaternion<f64>,
}

impl WorldPose {
    fn new(position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Self {
        Self {
            position,
            orientation,
        }
    }

    fn transform_point(self, point: Vector3<f64>) -> Vector3<f64> {
        self.position + self.orientation.transform_vector(&point)
    }
}

fn body_segments(origin: Vector3<f64>, points: &[ScenePoint]) -> Vec<SceneSegment> {
    if let [point] = points {
        return vec![SceneSegment {
            start: origin,
            end: point.position,
        }];
    }

    points
        .windows(2)
        .map(|points| SceneSegment {
            start: points[0].position,
            end: points[1].position,
        })
        .collect()
}

#[derive(Default)]
struct BoundsBuilder {
    min: Option<Vector2<f64>>,
    max: Option<Vector2<f64>>,
}

impl BoundsBuilder {
    fn include(&mut self, point: Vector2<f64>) {
        self.min = Some(self.min.map_or(point, |min| min.inf(&point)));
        self.max = Some(self.max.map_or(point, |max| max.sup(&point)));
    }

    fn finish(self) -> Option<ProjectedBounds> {
        let mut min = self.min?;
        let mut max = self.max?;
        let width = max.x - min.x;
        let height = max.y - min.y;

        if width <= f64::EPSILON {
            let half_extent = if height > f64::EPSILON {
                height * 0.05
            } else {
                0.5
            };
            min.x -= half_extent;
            max.x += half_extent;
        }
        if height <= f64::EPSILON {
            let half_extent = if width > f64::EPSILON {
                width * 0.05
            } else {
                0.5
            };
            min.y -= half_extent;
            max.y += half_extent;
        }

        Some(ProjectedBounds { min, max })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::yaml::parse_yaml_str;
    use crate::model::{Bodies, Body, Joints, Point};
    use crate::problem::prepare;
    use crate::solve::solve_at;

    const ONE_BODY: &str = include_str!("../../tests/fixtures/spherical_one_body_parse.yaml");
    const TWO_BODY: &str = include_str!("../../tests/fixtures/spherical_two_body_parse.yaml");

    #[test]
    fn projects_all_orthographic_planes() {
        let point = Vector3::new(1.0, 2.0, 3.0);

        assert_eq!(Projection::Xy.project(point), Vector2::new(1.0, 2.0));
        assert_eq!(Projection::Xz.project(point), Vector2::new(1.0, 3.0));
        assert_eq!(Projection::Yz.project(point), Vector2::new(2.0, 3.0));
    }

    #[test]
    fn projects_isometric_axes_with_z_up() {
        let x = Projection::Isometric.project(Vector3::x());
        let y = Projection::Isometric.project(Vector3::y());
        let z = Projection::Isometric.project(Vector3::z());

        assert!((x.x - 1.0 / 2.0_f64.sqrt()).abs() < 1.0e-12);
        assert!((y.x + 1.0 / 2.0_f64.sqrt()).abs() < 1.0e-12);
        assert!(z.x.abs() < 1.0e-12);
        assert!(x.y < 0.0);
        assert!(y.y < 0.0);
        assert!(z.y > 0.0);
    }

    #[test]
    fn builds_reference_geometry_from_body_points_and_joint_markers() {
        let input = parse_yaml_str(TWO_BODY).unwrap().into_input().unwrap();
        let scene = Scene::from_reference(input.model());

        assert_eq!(scene.bodies().len(), 2);
        assert_eq!(scene.joints().len(), 2);
        assert_eq!(scene.ground().origin(), Vector3::zeros());
        assert!(scene.ground().points().is_empty());
        assert!(scene.ground().segments().is_empty());
        assert_eq!(scene.ground().markers(), &[Vector3::zeros()]);

        let first_body = &scene.bodies()[0];
        assert_eq!(first_body.label(), "B1");
        assert_eq!(first_body.origin(), Vector3::new(0.0, 0.0, -100.0));
        assert_eq!(first_body.points()[0].position(), Vector3::zeros());
        assert_eq!(
            first_body.points()[1].position(),
            Vector3::new(0.0, 0.0, -200.0)
        );
        assert_eq!(first_body.segments().len(), 1);
        assert_eq!(first_body.segments()[0].start(), Vector3::zeros());
        assert_eq!(
            first_body.segments()[0].end(),
            Vector3::new(0.0, 0.0, -200.0)
        );

        let second_body = &scene.bodies()[1];
        assert_eq!(second_body.segments().len(), 1);
        assert_eq!(
            second_body.segments()[0].start(),
            Vector3::new(0.0, 0.0, -300.0)
        );
        assert_eq!(
            second_body.segments()[0].end(),
            Vector3::new(0.0, 0.0, -200.0)
        );

        assert_eq!(scene.joints()[0].i_position(), Vector3::zeros());
        assert_eq!(scene.joints()[0].j_position(), Vector3::zeros());
    }

    #[test]
    fn preserves_declared_ground_geometry_and_label() {
        let ground = Body::new(
            BodyId::GROUND,
            "foundation",
            Vector3::new(1.0, 2.0, 3.0),
            UnitQuaternion::identity(),
            vec![Point::new("anchor", Vector3::new(4.0, 0.0, 0.0))],
        );
        let model = crate::model::Model::new(
            Bodies::new(vec![ground]).unwrap(),
            Joints::new(vec![]).unwrap(),
        )
        .unwrap();
        let scene = Scene::from_reference(&model);

        assert_eq!(scene.ground().label(), "foundation");
        assert_eq!(scene.ground().points()[0].label(), "anchor");
        assert_eq!(
            scene.ground().points()[0].position(),
            Vector3::new(5.0, 2.0, 3.0)
        );
        assert_eq!(scene.ground().segments().len(), 1);
        assert_eq!(
            scene.projected_bounds(Projection::Xy).max(),
            Vector2::new(5.0, 2.2)
        );
    }

    #[test]
    fn builds_solved_geometry_from_body_poses() {
        let input = parse_yaml_str(ONE_BODY).unwrap().into_input().unwrap();
        let problem = prepare(input).unwrap();
        let poses = solve_at(&problem, 0.0).unwrap();
        let scene = Scene::from_body_poses(problem.model(), &poses).unwrap();
        let body = &scene.bodies()[0];
        let pose = poses.get(body.body_id()).unwrap();

        assert_eq!(body.origin(), pose.position());
        assert_eq!(
            body.points()[0].position(),
            pose.position()
                + pose.orientation().transform_vector(
                    &problem
                        .model()
                        .bodies()
                        .get(body.body_id())
                        .unwrap()
                        .points()[0]
                        .position()
                )
        );
    }

    #[test]
    fn gives_degenerate_scenes_usable_bounds() {
        let input = parse_yaml_str("hardpoints: {}\n")
            .unwrap()
            .into_input()
            .unwrap();
        let bounds = Scene::from_reference(input.model()).projected_bounds(Projection::Xz);

        assert_eq!(bounds.min(), Vector2::new(-0.5, -0.5));
        assert_eq!(bounds.max(), Vector2::new(0.5, 0.5));
    }

    #[test]
    fn combines_bounds_across_frames() {
        let first = scene_at(Vector3::new(-2.0, 1.0, 0.0));
        let second = scene_at(Vector3::new(4.0, 3.0, 0.0));
        let scenes = [first, second];
        let bounds = ProjectedBounds::for_scenes(&scenes, Projection::Xy).unwrap();

        assert_eq!(bounds.min(), Vector2::new(-2.0, 0.0));
        assert_eq!(bounds.max(), Vector2::new(4.0, 3.0));
    }

    #[test]
    fn fits_bounds_to_viewport_aspect_ratio() {
        let bounds = ProjectedBounds {
            min: Vector2::new(0.0, 0.0),
            max: Vector2::new(4.0, 2.0),
        };

        let square = bounds.fit_aspect(100, 100);
        assert_eq!(square.min(), Vector2::new(0.0, -1.0));
        assert_eq!(square.max(), Vector2::new(4.0, 3.0));

        let wide = bounds.fit_aspect(200, 100);
        assert_eq!(wide, bounds);
    }

    #[test]
    fn adds_fractional_padding() {
        let bounds = ProjectedBounds {
            min: Vector2::new(0.0, -1.0),
            max: Vector2::new(10.0, 9.0),
        }
        .padded(0.1);

        assert_eq!(bounds.min(), Vector2::new(-1.0, -2.0));
        assert_eq!(bounds.max(), Vector2::new(11.0, 10.0));
    }

    fn scene_at(origin: Vector3<f64>) -> Scene {
        Scene {
            bodies: vec![SceneBody {
                body_id: BodyId::new(1),
                label: "body".to_owned(),
                origin,
                points: vec![],
                segments: vec![],
            }],
            joints: vec![],
            ground: SceneGround {
                label: "ground".to_owned(),
                origin: Vector3::zeros(),
                points: vec![],
                segments: vec![],
                markers: vec![],
            },
        }
    }
}
