pub mod config;
pub mod coordinates;
pub mod mechanism;
pub mod motion;
pub mod spatial;
pub mod topology;
pub mod tree_step;

use thiserror::Error;

use config::SolverConfig;
use coordinates::{
    CoordinateLayout, JointCoordinateError, JointCoordinates, resolve_joint_coordinates,
};
use mechanism::joint::spherical::SphericalCoordinate;
use mechanism::{JointId, Mechanism};
use motion::Motions;
use topology::{Topology, TopologyError};
use tree_step::TreeStep;

/// Fully built model: mechanism, motions, solver settings, and the derived
/// topology, joint coordinates, and coordinate layout.
///
/// Immutable after construction.
pub struct Model {
    mechanism: Mechanism,
    motions: Motions,
    solver: SolverConfig,
    joint_coordinates: JointCoordinates,
    tree_steps: Vec<TreeStep>,
    closure_joint_ids: Vec<JointId>,
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

        let tree_steps = topology
            .tree_edges()
            .iter()
            .map(|edge| {
                let coordinate = joint_coordinates
                    .get(edge.joint_id())
                    .expect("topology references a mechanism joint");

                TreeStep::new(
                    edge.parent_body_id(),
                    edge.child_body_id(),
                    edge.joint_id(),
                    *coordinate,
                    edge.direction(),
                )
            })
            .collect::<Vec<_>>();

        let primary_coordinates = tree_steps
            .iter()
            .flat_map(|step| {
                (0..step.joint_coordinate().component_count())
                    .map(move |component| (step.joint_id(), component))
            })
            .collect();
        let free_primary_coordinates = tree_steps
            .iter()
            .flat_map(|step| {
                step.joint_coordinate()
                    .free_component_indices()
                    .into_iter()
                    .map(move |component| (step.joint_id(), component))
            })
            .collect();

        Ok(Self {
            mechanism,
            motions,
            solver,
            joint_coordinates,
            tree_steps,
            closure_joint_ids: topology.closure_joint_ids().to_vec(),
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

    /// Returns spanning-tree steps in parent-before-child order.
    pub fn tree_edges(&self) -> &[TreeStep] {
        &self.tree_steps
    }

    /// Returns joints that close loops in the mechanism.
    pub fn closure_joint_ids(&self) -> &[JointId] {
        &self.closure_joint_ids
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
