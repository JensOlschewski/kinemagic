use std::collections::{BTreeMap, BTreeSet, VecDeque};

use thiserror::Error;

use crate::model::{BodyId, JointId, JointRole, Model};

/// Topological representation of the mechanism.
///
/// The mechanism is decomposed into:
///
///  - a rooted spanning tree used for forward traversal from ground, and
///  - closure joints representing edges that would introduce cycles.
///
/// For an open-chain mechanism, `closure_joint_ids` is empty.
/// For a closed-loop mechanism, one or more joints are represented as
/// closure joints.
#[derive(Debug, Eq, PartialEq)]
pub struct KinematicTopology {
    tree_edges: Vec<TreeEdge>,
    closure_joint_ids: Vec<JointId>,
}

impl KinematicTopology {
    /// Returns the spanning-tree edges in parent-before-child order.
    pub fn tree_edges(&self) -> &[TreeEdge] {
        &self.tree_edges
    }

    /// Returns joints that close loops in the mechanism.
    pub fn closure_joint_ids(&self) -> &[JointId] {
        &self.closure_joint_ids
    }
}

/// An oriented edge of the spanning tree.
///
/// Parent and child refer to the tree orientation and are independent of
/// the joint's I/J marker ordering.
#[derive(Debug, Eq, PartialEq)]
pub struct TreeEdge {
    parent_body_id: BodyId,
    child_body_id: BodyId,
    joint_id: JointId,
    direction: TraversalDirection,
}

impl TreeEdge {
    pub fn parent_body_id(&self) -> BodyId {
        self.parent_body_id
    }

    pub fn child_body_id(&self) -> BodyId {
        self.child_body_id
    }

    pub fn joint_id(&self) -> JointId {
        self.joint_id
    }

    pub fn direction(&self) -> TraversalDirection {
        self.direction
    }
}

/// Direction in which a joint is traversed relative to its I/J markers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraversalDirection {
    IToJ,
    JToI,
}

/// Result of partitioning the mechanism joints into a spanning tree
/// and loop-closing edges.
#[derive(Debug)]
pub struct JointPartition {
    tree_joint_ids: Vec<JointId>,
    closure_joint_ids: Vec<JointId>,
}

/// Tracks connected components while constructing the spanning tree.
///
/// `union(a, b)` returns `true` when the components were previously
/// disconnected and have now been joined. It returns `false` when both
/// items already belong to the same component, meaning that adding the
/// corresponding edge would create a cycle.
struct UnionFind {
    parents: Vec<usize>,
}

impl UnionFind {
    fn new(size: usize) -> Self {
        Self {
            parents: (0..size).collect(),
        }
    }

    fn find(&self, mut item: usize) -> usize {
        while self.parents[item] != item {
            item = self.parents[item];
        }

        item
    }

    /// Joins the sets containing `a` and `b`.
    ///
    /// Returns `true` if two previously separate sets were joined,
    /// or `false` if `a` and `b` were already connected.
    fn union(&mut self, a: usize, b: usize) -> bool {
        let a_root = self.find(a);
        let b_root = self.find(b);

        if a_root == b_root {
            return false;
        }

        self.parents[b_root] = a_root;
        true
    }
}

fn partition_tree_and_closure_joints(
    model: &Model,
) -> Result<JointPartition, KinematicTopologyError> {
    let mut body_ids = model
        .bodies()
        .iter()
        .map(|body| body.id())
        .collect::<Vec<_>>();
    body_ids.sort_unstable();

    let body_indices = body_ids
        .into_iter()
        .enumerate()
        .map(|(index, body_id)| (body_id, index))
        .collect::<BTreeMap<_, _>>();

    let mut joints = model.joints().iter().collect::<Vec<_>>();
    let mut connectivity = UnionFind::new(body_indices.len());
    let mut tree_joint_ids = Vec::new();
    let mut closure_joint_ids = Vec::new();

    joints.sort_unstable_by_key(|joint| {
        let priority = match joint.role() {
            JointRole::Primary => 0,
            JointRole::Auto | JointRole::Secondary => 1,
        };

        (priority, joint.id())
    });

    for joint in joints {
        let i_body_id = joint.i_marker().body_id();
        let j_body_id = joint.j_marker().body_id();

        match joint.role() {
            JointRole::Primary => {
                let i_index = body_indices[&i_body_id];
                let j_index = body_indices[&j_body_id];

                if !connectivity.union(i_index, j_index) {
                    return Err(KinematicTopologyError::TreeJointCycle {
                        joint_id: joint.id(),
                    });
                }

                tree_joint_ids.push(joint.id());
            }

            JointRole::Auto => {
                let i_index = body_indices[&i_body_id];
                let j_index = body_indices[&j_body_id];

                if connectivity.union(i_index, j_index) {
                    tree_joint_ids.push(joint.id());
                } else {
                    closure_joint_ids.push(joint.id());
                }
            }

            JointRole::Secondary => {
                closure_joint_ids.push(joint.id());
            }
        }
    }

    Ok(JointPartition {
        tree_joint_ids,
        closure_joint_ids,
    })
}

#[derive(Clone, Copy, Debug)]
struct AdjacencyEdge {
    neighbor_body_id: BodyId,
    joint_id: JointId,
    direction: TraversalDirection,
}

fn orient_tree_edges(
    model: &Model,
    tree_joint_ids: &[JointId],
) -> Result<Vec<TreeEdge>, KinematicTopologyError> {
    let tree_joint_ids = tree_joint_ids.iter().copied().collect::<BTreeSet<_>>();

    let mut adjacency = BTreeMap::<BodyId, Vec<AdjacencyEdge>>::new();

    for joint in model
        .joints()
        .iter()
        .filter(|joint| tree_joint_ids.contains(&joint.id()))
    {
        let joint_id = joint.id();
        let i_body_id = joint.i_marker().body_id();
        let j_body_id = joint.j_marker().body_id();

        // I -> J
        adjacency.entry(i_body_id).or_default().push(AdjacencyEdge {
            neighbor_body_id: j_body_id,
            joint_id,
            direction: TraversalDirection::IToJ,
        });

        // J -> I
        adjacency.entry(j_body_id).or_default().push(AdjacencyEdge {
            neighbor_body_id: i_body_id,
            joint_id,
            direction: TraversalDirection::JToI,
        });
    }

    // Sort neighboring edges to ensure deterministic breadth-first traversal.
    // First sort by neighbor body ID, then by joint ID.
    for edges in adjacency.values_mut() {
        edges.sort_unstable_by_key(|edge| (edge.neighbor_body_id, edge.joint_id));
    }

    let mut queue = VecDeque::from([BodyId::GROUND]);
    let mut visited = BTreeSet::from([BodyId::GROUND]);
    let mut tree_edges = Vec::new();

    while let Some(parent_body_id) = queue.pop_front() {
        for edge in adjacency.get(&parent_body_id).into_iter().flatten() {
            if visited.insert(edge.neighbor_body_id) {
                tree_edges.push(TreeEdge {
                    parent_body_id,
                    child_body_id: edge.neighbor_body_id,
                    joint_id: edge.joint_id,
                    direction: edge.direction,
                });

                queue.push_back(edge.neighbor_body_id);
            }
        }
    }

    // Every body must be reachable through the selected tree joints.
    if let Some(body) = model
        .bodies()
        .iter()
        .find(|body| !visited.contains(&body.id()))
    {
        return Err(KinematicTopologyError::UnreachableTreeBody { body_id: body.id() });
    }

    Ok(tree_edges)
}

pub fn build_kinematic_topology(
    model: &Model,
) -> Result<KinematicTopology, KinematicTopologyError> {
    let partition = partition_tree_and_closure_joints(model)?;
    let tree_edges = orient_tree_edges(model, &partition.tree_joint_ids)?;

    Ok(KinematicTopology {
        tree_edges,
        closure_joint_ids: partition.closure_joint_ids,
    })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum KinematicTopologyError {
    #[error("tree joint `{joint_id:?}` would create a cycle")]
    TreeJointCycle { joint_id: JointId },
    #[error("body `{body_id:?}` is not reachable from ground through tree joints")]
    UnreachableTreeBody { body_id: BodyId },
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::model::*;

    #[test]
    fn builds_ground_only_topology() {
        let model = model(&[], vec![]);

        let topology = build_kinematic_topology(&model).unwrap();

        assert!(topology.tree_edges().is_empty());
        assert!(topology.closure_joint_ids().is_empty());
    }

    #[test]
    fn builds_single_tree_edge() {
        let model = model(&[1], vec![joint(1, 0, 1)]);

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[tree_edge(0, 1, 1, TraversalDirection::IToJ)]
        );

        assert!(topology.closure_joint_ids().is_empty());
    }

    #[test]
    fn orients_tree_edge_from_ground() {
        // Joint orientation is body 1 (I) -> ground (J),
        // but tree traversal must still start at ground.
        let model = model(&[1], vec![joint(1, 1, 0)]);

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[tree_edge(0, 1, 1, TraversalDirection::JToI)]
        );
    }

    #[test]
    fn builds_open_chain_in_tree_order() {
        let model = model(&[1, 2], vec![joint(2, 1, 2), joint(1, 0, 1)]);

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[
                tree_edge(0, 1, 1, TraversalDirection::IToJ),
                tree_edge(1, 2, 2, TraversalDirection::IToJ),
            ]
        );
    }

    #[test]
    fn builds_branch_in_breadth_first_order() {
        let model = model(
            &[1, 2, 3],
            vec![joint(3, 1, 3), joint(2, 0, 2), joint(1, 0, 1)],
        );

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[
                tree_edge(0, 1, 1, TraversalDirection::IToJ),
                tree_edge(0, 2, 2, TraversalDirection::IToJ),
                tree_edge(1, 3, 3, TraversalDirection::IToJ),
            ]
        );
    }

    #[test]
    fn represents_closed_loop_with_closure_joint() {
        // Two joints connect ground and body 1.
        //
        // J1 becomes part of the spanning tree.
        // J2 would create a loop and therefore becomes a closure joint.
        let model = model(&[1], vec![joint(2, 1, 0), joint(1, 0, 1)]);

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[tree_edge(0, 1, 1, TraversalDirection::IToJ)]
        );

        assert_eq!(topology.closure_joint_ids(), &[JointId::new(2)]);
    }

    fn model(body_ids: &[u32], joints: Vec<Joint>) -> Model {
        let bodies = std::iter::once(BodyId::GROUND)
            .chain(body_ids.iter().copied().map(BodyId::new))
            .map(body)
            .collect();

        Model::new(Bodies::new(bodies).unwrap(), Joints::new(joints).unwrap()).unwrap()
    }

    #[test]
    fn primary_joint_is_preferred_over_auto_joint() {
        let model = model(
            &[1],
            vec![
                joint_with_role(1, 0, 1, JointRole::Auto),
                joint_with_role(2, 0, 1, JointRole::Primary),
            ],
        );

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[tree_edge(0, 1, 2, TraversalDirection::IToJ)]
        );

        assert_eq!(topology.closure_joint_ids(), &[JointId::new(1)]);
    }

    #[test]
    fn rejects_cycle_of_primary_joints() {
        let model = model(
            &[1, 2],
            vec![
                joint_with_role(1, 0, 1, JointRole::Primary),
                joint_with_role(2, 1, 2, JointRole::Primary),
                joint_with_role(3, 2, 0, JointRole::Primary),
            ],
        );

        let error = build_kinematic_topology(&model).unwrap_err();

        assert!(matches!(
            error,
            KinematicTopologyError::TreeJointCycle { joint_id }
                if joint_id == JointId::new(3)
        ));
    }

    fn body(id: BodyId) -> Body {
        Body::new(
            id,
            format!("body {id:?}"),
            Vector3::zeros(),
            UnitQuaternion::identity(),
            vec![],
        )
    }

    #[test]
    fn secondary_joint_becomes_closure_joint() {
        let model = model(
            &[1],
            vec![
                joint_with_role(1, 0, 1, JointRole::Primary),
                joint_with_role(2, 0, 1, JointRole::Secondary),
            ],
        );

        let topology = build_kinematic_topology(&model).unwrap();

        assert_eq!(
            topology.tree_edges(),
            &[tree_edge(0, 1, 1, TraversalDirection::IToJ)]
        );

        assert_eq!(topology.closure_joint_ids(), &[JointId::new(2)]);
    }

    #[test]
    fn rejects_tree_disconnected_by_secondary_joint() {
        let model = model(&[1], vec![joint_with_role(1, 0, 1, JointRole::Secondary)]);

        let error = build_kinematic_topology(&model).unwrap_err();

        assert!(matches!(
            error,
            KinematicTopologyError::UnreachableTreeBody { body_id }
                if body_id == BodyId::new(1)
        ));
    }

    fn joint(id: u32, i_body_id: u32, j_body_id: u32) -> Joint {
        joint_with_role(id, i_body_id, j_body_id, JointRole::Auto)
    }

    fn joint_with_role(id: u32, i_body_id: u32, j_body_id: u32, role: JointRole) -> Joint {
        Joint::new(
            JointId::new(id),
            format!("joint {id}"),
            JointKind::Spherical,
            role,
            marker("i", BodyId::new(i_body_id)),
            marker("j", BodyId::new(j_body_id)),
        )
    }

    fn marker(name: &str, body_id: BodyId) -> Marker {
        Marker::new(name, body_id, Vector3::zeros(), UnitQuaternion::identity())
    }

    fn tree_edge(
        parent_body_id: u32,
        child_body_id: u32,
        joint_id: u32,
        direction: TraversalDirection,
    ) -> TreeEdge {
        TreeEdge {
            parent_body_id: BodyId::new(parent_body_id),
            child_body_id: BodyId::new(child_body_id),
            joint_id: JointId::new(joint_id),
            direction,
        }
    }
}
