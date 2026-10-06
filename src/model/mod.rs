pub mod config;
pub mod coordinates;
pub mod mechanism;
pub mod motion;
pub mod spatial;
pub mod topology;

use thiserror::Error;

use config::SolverConfig;
use coordinates::{
    CoordinateLayout, JointCoordinateError, JointCoordinates, resolve_joint_coordinates,
};
use mechanism::joint::spherical::SphericalCoordinate;
use mechanism::{JointId, Mechanism};
use motion::Motions;
use topology::{Topology, TopologyError, TreeEdge};

/// Fully built model: mechanism, motions, solver settings, and the derived
/// topology, joint coordinates, and coordinate layout.
///
/// Immutable after construction.
pub struct Model {
    mechanism: Mechanism,
    motions: Motions,
    solver: SolverConfig,
    joint_coordinates: JointCoordinates,
    topology: Topology,
    coordinate_layout: CoordinateLayout,
}

impl Model {
    /// Creates a model using [`SolverConfig::defaults`].
    ///
    /// # Errors
    ///
    /// Returns [`ModelError`] if topology or joint coordinates are invalid.
    pub fn new(mechanism: Mechanism, motions: Motions) -> Result<Self, ModelError> {
        Self::with_solver(mechanism, motions, SolverConfig::defaults())
    }

    /// Creates a model with an explicit solver time grid.
    ///
    /// # Errors
    ///
    /// Returns [`ModelError`] if topology or joint coordinates are invalid.
    pub fn with_solver(
        mechanism: Mechanism,
        motions: Motions,
        solver: SolverConfig,
    ) -> Result<Self, ModelError> {
        let topology = Topology::build(&mechanism)?;
        let joint_coordinates = resolve_joint_coordinates(&mechanism, &motions)?;

        let primary_coordinates = topology
            .tree_edges()
            .iter()
            .flat_map(|edge| {
                let coordinate = joint_coordinates
                    .get(edge.joint_id())
                    .expect("topology references a mechanism joint");
                (0..coordinate.component_count()).map(move |component| (edge.joint_id(), component))
            })
            .collect();
        let free_primary_coordinates = topology
            .tree_edges()
            .iter()
            .flat_map(|edge| {
                joint_coordinates
                    .get(edge.joint_id())
                    .expect("topology references a mechanism joint")
                    .free_component_indices()
                    .into_iter()
                    .map(move |component| (edge.joint_id(), component))
            })
            .collect();

        Ok(Self {
            mechanism,
            motions,
            solver,
            joint_coordinates,
            topology,
            coordinate_layout: CoordinateLayout::new(primary_coordinates, free_primary_coordinates),
        })
    }

    /// Returns the mechanism: bodies, markers, and joints.
    pub fn mechanism(&self) -> &Mechanism {
        &self.mechanism
    }

    /// Returns prescribed joint motions.
    pub fn motions(&self) -> &Motions {
        &self.motions
    }

    /// Returns the solver time-grid configuration.
    pub fn solver(&self) -> SolverConfig {
        self.solver
    }

    pub fn joint_coordinate(&self, joint_id: JointId) -> Option<&SphericalCoordinate> {
        self.joint_coordinates.get(joint_id)
    }

    /// Returns the rooted topology of the mechanism.
    pub fn topology(&self) -> &Topology {
        &self.topology
    }

    /// Returns spanning-tree edges in parent-before-child order.
    pub fn tree_edges(&self) -> &[TreeEdge] {
        self.topology.tree_edges()
    }

    /// Returns joints that close loops in the mechanism.
    pub fn closure_joint_ids(&self) -> &[JointId] {
        self.topology.closure_joint_ids()
    }

    pub fn free_primary_coordinates(&self) -> Vec<(JointId, usize)> {
        self.coordinate_layout.free_primary_coordinates().to_vec()
    }

    pub fn primary_coordinates(&self) -> Vec<(JointId, usize)> {
        self.coordinate_layout.primary_coordinates().to_vec()
    }

    pub fn primary_coordinate_count(&self) -> usize {
        self.coordinate_layout.primary_coordinate_count()
    }

    pub fn coordinate_layout(&self) -> &CoordinateLayout {
        &self.coordinate_layout
    }
}

/// Errors returned when deriving topology and coordinates for a model.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ModelError {
    #[error(transparent)]
    Topology(#[from] TopologyError),
    #[error(transparent)]
    Coordinates(#[from] JointCoordinateError),
}
