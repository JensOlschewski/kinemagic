use std::collections::{BTreeMap, BTreeSet, VecDeque};

use thiserror::Error;

use crate::model::{BodyId, JointId, Model};

#[derive(Debug, Eq, PartialEq)]
pub struct KinematicTree {
    steps: Vec<KinematicTreeStep>,
}

impl KinematicTree {
    pub fn steps(&self) -> &[KinematicTreeStep] {
        &self.steps
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct KinematicTreeStep {
    pub parent_id: BodyId,
    pub child_id: BodyId,
    pub joint_id: JointId,
}

type Adjacency = BTreeMap<BodyId, Vec<Edge>>;

#[derive(Clone, Copy)]
struct Edge {
    child_id: BodyId,
    joint_id: JointId,
}

pub fn build_kinematic_tree(model: &Model) -> Result<KinematicTree, KinematicTreeError> {
    let mut adjacency = Adjacency::new();
    let mut parent_by_child = BTreeMap::new();

    for joint in model.joints().iter() {
        let joint_id = joint.id();
        let parent_id = joint.i_marker().body_id();
        let child_id = joint.j_marker().body_id();

        if parent_id == child_id {
            return Err(KinematicTreeError::SelfParenting {
                body_id: parent_id,
                joint_id,
            });
        }

        if child_id == BodyId::GROUND {
            return Err(KinematicTreeError::GroundAsChild {
                parent_id,
                joint_id,
            });
        }

        if let Some((_, first_joint_id)) = parent_by_child.insert(child_id, (parent_id, joint_id)) {
            return Err(KinematicTreeError::DuplicateParent {
                body_id: child_id,
                first_joint_id,
                second_joint_id: joint_id,
            });
        }

        adjacency
            .entry(parent_id)
            .or_default()
            .push(Edge { child_id, joint_id });
    }

    for edges in adjacency.values_mut() {
        edges.sort_unstable_by_key(|edge| (edge.child_id, edge.joint_id));
    }

    let mut queue = VecDeque::from([BodyId::GROUND]);
    let mut visited = BTreeSet::from([BodyId::GROUND]);
    let mut steps = Vec::new();

    while let Some(parent_id) = queue.pop_front() {
        for edge in adjacency.get(&parent_id).into_iter().flatten() {
            if visited.insert(edge.child_id) {
                steps.push(KinematicTreeStep {
                    parent_id,
                    child_id: edge.child_id,
                    joint_id: edge.joint_id,
                });
                queue.push_back(edge.child_id);
            }
        }
    }

    if let Some(body) = model
        .bodies()
        .iter()
        .find(|body| !visited.contains(&body.id()))
    {
        return Err(KinematicTreeError::UnreachableBody { body_id: body.id() });
    }

    Ok(KinematicTree { steps })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum KinematicTreeError {
    #[error("joint `{joint_id:?}` makes body `{body_id:?}` its own parent")]
    SelfParenting { body_id: BodyId, joint_id: JointId },
    #[error("joint `{joint_id:?}` makes ground a child of body `{parent_id:?}`")]
    GroundAsChild {
        parent_id: BodyId,
        joint_id: JointId,
    },
    #[error("body `{body_id:?}` has parent joints `{first_joint_id:?}` and `{second_joint_id:?}`")]
    DuplicateParent {
        body_id: BodyId,
        first_joint_id: JointId,
        second_joint_id: JointId,
    },
    #[error("body `{body_id:?}` is not reachable from ground through directed joints")]
    UnreachableBody { body_id: BodyId },
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::io::yaml::parse_yaml_str;
    use crate::model::{Bodies, Body, Joint, JointKind, Joints, Marker};

    #[test]
    fn builds_ground_only_tree() {
        let model = model(&[], vec![]);

        let tree = build_kinematic_tree(&model).unwrap();

        assert!(tree.steps().is_empty());
    }

    #[test]
    fn builds_one_body_tree() {
        let model = model(&[1], vec![joint(1, 0, 1)]);

        let tree = build_kinematic_tree(&model).unwrap();

        assert_eq!(tree.steps(), &[step(0, 1, 1)]);
    }

    #[test]
    fn builds_chain_in_topology_order() {
        let model = model(&[2, 1], vec![joint(1, 1, 2), joint(9, 0, 1)]);

        let tree = build_kinematic_tree(&model).unwrap();

        assert_eq!(tree.steps(), &[step(0, 1, 9), step(1, 2, 1)]);
    }

    #[test]
    fn builds_branch_in_deterministic_breadth_first_order() {
        let model = model(
            &[3, 2, 1],
            vec![joint(1, 1, 3), joint(9, 0, 2), joint(8, 0, 1)],
        );

        let tree = build_kinematic_tree(&model).unwrap();

        assert_eq!(tree.steps(), &[step(0, 1, 8), step(0, 2, 9), step(1, 3, 1)]);
    }

    #[test]
    fn yaml_name_order_does_not_change_steps() {
        let fixture = include_str!("../../tests/fixtures/spherical_two_body_parse.yaml");
        let root_first = fixture
            .replace("  J1:", "  ARoot:")
            .replace("joint_id: 1", "joint_id: 9")
            .replace("  J2:", "  ZChild:")
            .replace("joint_id: 2", "joint_id: 1");
        let child_first = fixture
            .replace("  J1:", "  ZRoot:")
            .replace("joint_id: 1", "joint_id: 9")
            .replace("  J2:", "  AChild:")
            .replace("joint_id: 2", "joint_id: 1");
        let root_first = parse_yaml_str(&root_first).unwrap().into_model().unwrap();
        let child_first = parse_yaml_str(&child_first).unwrap().into_model().unwrap();

        assert_eq!(
            build_kinematic_tree(&root_first).unwrap(),
            build_kinematic_tree(&child_first).unwrap()
        );
    }

    #[test]
    fn rejects_self_parenting() {
        let model = model(&[1], vec![joint(1, 1, 1), joint(2, 0, 1)]);

        let error = build_kinematic_tree(&model).unwrap_err();

        assert!(matches!(
            error,
            KinematicTreeError::SelfParenting { body_id, joint_id }
                if body_id == BodyId::new(1) && joint_id == JointId::new(1)
        ));
    }

    #[test]
    fn rejects_ground_as_child() {
        let model = model(&[1], vec![joint(1, 1, 0)]);

        let error = build_kinematic_tree(&model).unwrap_err();

        assert!(matches!(
            error,
            KinematicTreeError::GroundAsChild {
                parent_id,
                joint_id
            } if parent_id == BodyId::new(1) && joint_id == JointId::new(1)
        ));
    }

    #[test]
    fn rejects_duplicate_parent() {
        let model = model(&[1], vec![joint(1, 0, 1), joint(2, 0, 1)]);

        let error = build_kinematic_tree(&model).unwrap_err();

        assert!(matches!(
            error,
            KinematicTreeError::DuplicateParent {
                body_id,
                first_joint_id,
                second_joint_id,
            } if body_id == BodyId::new(1)
                && first_joint_id == JointId::new(1)
                && second_joint_id == JointId::new(2)
        ));
    }

    #[test]
    fn rejects_directed_cycle() {
        let model = model(
            &[1, 2],
            vec![joint(3, 0, 1), joint(2, 1, 2), joint(1, 2, 1)],
        );

        let error = build_kinematic_tree(&model).unwrap_err();

        assert!(matches!(
            error,
            KinematicTreeError::DuplicateParent {
                body_id,
                first_joint_id,
                second_joint_id,
            } if body_id == BodyId::new(1)
                && first_joint_id == JointId::new(3)
                && second_joint_id == JointId::new(1)
        ));
    }

    fn model(body_ids: &[u32], joints: Vec<Joint>) -> Model {
        let bodies = std::iter::once(BodyId::GROUND)
            .chain(body_ids.iter().copied().map(BodyId::new))
            .map(body)
            .collect();

        Model::new(Bodies::new(bodies).unwrap(), Joints::new(joints).unwrap()).unwrap()
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

    fn joint(id: u32, parent_id: u32, child_id: u32) -> Joint {
        Joint::new(
            JointId::new(id),
            format!("joint {id}"),
            JointKind::Spherical,
            marker("i", BodyId::new(parent_id)),
            marker("j", BodyId::new(child_id)),
        )
    }

    fn marker(name: &str, body_id: BodyId) -> Marker {
        Marker::new(name, body_id, Vector3::zeros(), UnitQuaternion::identity())
    }

    fn step(parent_id: u32, child_id: u32, joint_id: u32) -> KinematicTreeStep {
        KinematicTreeStep {
            parent_id: BodyId::new(parent_id),
            child_id: BodyId::new(child_id),
            joint_id: JointId::new(joint_id),
        }
    }
}
