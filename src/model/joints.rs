use nalgebra::{Rotation3, UnitQuaternion, Vector3};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use crate::model::{
    Bodies, BodiesError, BodyId, BodyPose, BuildOrientationError, Orientation, OrientationSpec,
    Side,
};

#[derive(Clone)]
pub struct Joint {
    name: String,
    id: JointId,
    kind: JointKind,
    topology: JointTopology,
    i_endpoint: JointEndpoint,
    j_endpoint: JointEndpoint,
}

impl Joint {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn id(&self) -> JointId {
        self.id
    }

    pub fn kind(&self) -> JointKind {
        self.kind
    }
    pub fn topology(&self) -> JointTopology {
        self.topology
    }

    pub fn i_endpoint(&self) -> &JointEndpoint {
        &self.i_endpoint
    }

    pub fn j_endpoint(&self) -> &JointEndpoint {
        &self.j_endpoint
    }

    /// Keys ensure that joints are unique based on their topology and id
    pub fn key(&self) -> JointKey {
        JointKey {
            topology: self.topology,
            id: self.id,
        }
    }

    pub fn from_spec(spec: &JointSpec, bodies: &Bodies) -> Result<Self, BuildJointsError> {
        let i_side = endpoint_lookup_side(&spec.i_endpoint, &spec.j_endpoint, bodies)?;
        let j_side = endpoint_lookup_side(&spec.j_endpoint, &spec.i_endpoint, bodies)?;

        Ok(Self {
            name: spec.name.clone(),
            id: spec.id,
            kind: spec.kind,
            topology: spec.topology,
            i_endpoint: JointEndpoint::from_spec(&spec.i_endpoint, bodies, i_side)?,
            j_endpoint: JointEndpoint::from_spec(&spec.j_endpoint, bodies, j_side)?,
        })
    }
}

#[derive(Default)]
pub struct Joints {
    primary: BTreeMap<JointId, Joint>,
    secondary: BTreeMap<JointId, Joint>,
}

impl Joints {
    pub fn new() -> Self {
        Self {
            primary: BTreeMap::new(),
            secondary: BTreeMap::new(),
        }
    }

    pub fn primary(&self) -> impl Iterator<Item = &Joint> {
        self.primary.values()
    }

    pub fn secondary(&self) -> impl Iterator<Item = &Joint> {
        self.secondary.values()
    }

    pub fn build(specs: &[JointSpec], bodies: &Bodies) -> Result<Self, BuildJointsError> {
        let mut joints = Joints::new();
        let mut used_keys = BTreeSet::new();

        for spec in specs {
            let joint = Joint::from_spec(spec, bodies)?;
            let key = joint.key();

            if !used_keys.insert(key) {
                return Err(BuildJointsError::DuplicateJointKey(key));
            }

            match joint.topology {
                JointTopology::Primary => {
                    joints.primary.insert(joint.id(), joint);
                }

                JointTopology::Secondary => {
                    joints.secondary.insert(joint.id(), joint);
                }
            }
        }

        Ok(joints)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct JointId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct JointKey {
    pub topology: JointTopology,
    pub id: JointId,
}

#[derive(Clone, Copy, Debug)]
pub enum JointKind {
    Revolute,
    Spherical,
    Universal,
    Prismatic,
    Planar,
    Inplane,
    Inline,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum JointTopology {
    Primary,
    Secondary,
}

#[derive(Clone)]
pub struct JointEndpoint {
    pub body_id: BodyId,
    pub local_marker: JointMarker,
}

impl JointEndpoint {
    pub fn from_spec(
        spec: &JointEndpointSpec,
        bodies: &Bodies,
        point_side: Side,
    ) -> Result<Self, BuildJointsError> {
        let body = bodies
            .get_by_id(spec.body_id)
            .ok_or(BuildJointsError::MissingBody(spec.body_id))?;

        let global_position = bodies
            .global_point(spec.body_id, &spec.point, point_side)
            .map_err(|source| BuildJointsError::InvalidBody { source })?;

        let global_orientation = Orientation::from_spec(&spec.orientation, |point_name| {
            bodies
                .global_point(spec.body_id, point_name, point_side)
                .ok()
        })
        .map_err(|source| BuildJointsError::InvalidOrientation {
            body_id: spec.body_id,
            source,
        })?
        .rotation_matrix();

        let local_marker =
            JointMarker::from_global(global_position, global_orientation, body.pose());

        Ok(Self {
            body_id: spec.body_id,
            local_marker,
        })
    }
}

/* Marker is always defined in body-fixed local refercene frame
in this way marker does not need to be updated just transformed
based on body pose */
#[derive(Clone)]
pub struct JointMarker {
    pub local_position: Vector3<f64>,
    pub local_orientation: Rotation3<f64>,
}

impl JointMarker {
    fn from_global(
        position: Vector3<f64>,
        orientation: Rotation3<f64>,
        body_pose: &BodyPose,
    ) -> Self {
        let body_rotation = body_pose.orientation.to_rotation_matrix();

        Self {
            local_position: body_pose.global_to_local(position),
            local_orientation: body_rotation.inverse() * orientation,
        }
    }

    pub fn to_unit_quaternion(&self) -> nalgebra::UnitQuaternion<f64> {
        UnitQuaternion::from_rotation_matrix(&self.local_orientation)
    }

    pub fn to_global_unit_quaternion(&self, body_pose: &BodyPose) -> UnitQuaternion<f64> {
        body_pose.orientation * self.to_unit_quaternion()
    }

    pub fn global_position(&self, body_pose: &BodyPose) -> Vector3<f64> {
        body_pose.local_to_global(self.local_position)
    }

    pub fn global_orientation(&self, body_pose: &BodyPose) -> Rotation3<f64> {
        body_pose.orientation.to_rotation_matrix() * self.local_orientation
    }
}

#[derive(Clone)]
pub struct JointSpec {
    pub name: String,
    pub id: JointId,
    pub kind: JointKind,
    pub topology: JointTopology,
    pub i_endpoint: JointEndpointSpec,
    pub j_endpoint: JointEndpointSpec,
}

#[derive(Clone)]
pub struct JointEndpointSpec {
    pub body_id: BodyId,
    pub point: String,
    pub orientation: OrientationSpec,
}

#[derive(Debug)]
pub enum BuildJointsError {
    DuplicateJointKey(JointKey),
    MissingBody(BodyId),
    InvalidBody {
        source: BodiesError,
    },
    InvalidOrientation {
        body_id: BodyId,
        source: BuildOrientationError,
    },
}

impl fmt::Display for BuildJointsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingBody(body_id) => {
                write!(f, "body defined by: '{body_id:?}' is not defined in bodies")
            }
            Self::InvalidBody { source } => {
                write!(f, "invalid joint body or point: {source}")
            }
            Self::InvalidOrientation { body_id, source } => {
                write!(f, "invalid orientation for body {body_id:?}: {source}")
            }
            Self::DuplicateJointKey(key) => {
                write!(f, "duplicate {:?} joint id {}", key.topology, key.id.0)
            }
        }
    }
}

impl Error for BuildJointsError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidBody { source } => Some(source),
            Self::InvalidOrientation { source, .. } => Some(source),
            _ => None,
        }
    }
}

fn endpoint_lookup_side(
    endpoint: &JointEndpointSpec,
    other_endpoint: &JointEndpointSpec,
    bodies: &Bodies,
) -> Result<Side, BuildJointsError> {
    if endpoint.body_id == BodyId::GROUND {
        if other_endpoint.body_id == BodyId::GROUND {
            return Ok(Side::Single);
        }

        return bodies
            .get_by_id(other_endpoint.body_id)
            .map(|body| body.side())
            .ok_or(BuildJointsError::MissingBody(other_endpoint.body_id));
    }

    bodies
        .get_by_id(endpoint.body_id)
        .map(|body| body.side())
        .ok_or(BuildJointsError::MissingBody(endpoint.body_id))
}
