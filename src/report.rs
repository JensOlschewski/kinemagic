//! Narrative text report of a [`Model`]: counts, loop structure, and
//! solver settings, for `km check` to show a human before solving.
//!
//! Parallel to [`crate::view`]'s responsibility (take a `Model`, produce
//! something to show a human), just rendered as text instead of a scene.

use std::collections::BTreeMap;
use std::fmt;

use crate::model::Model;
use crate::model::config::SolverConfig;
use crate::model::mechanism::{JointId, JointKind};

/// Narrative summary of a model's bodies, joints, coordinates, motions,
/// solver settings, and topology.
#[derive(Debug, Clone, PartialEq)]
pub struct ModelReport {
    body_count: usize,
    joint_count: usize,
    joint_kind_counts: BTreeMap<JointKind, usize>,
    primary_coordinate_count: usize,
    free_coordinate_count: usize,
    motion_count: usize,
    solver: SolverConfig,
    topology: TopologyReport,
}

/// Open-tree or closed-loop structure of a model.
#[derive(Debug, Clone, PartialEq)]
pub enum TopologyReport {
    OpenTree,
    ClosedLoop { loops: Vec<LoopReport> },
}

/// One fundamental loop: a closure joint plus the tree path it closes.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopReport {
    closure_joint_id: JointId,
    /// Tree-path joints from each of `closure_joint_id`'s two bodies up to
    /// their lowest common ancestor, concatenated, closure joint appended last.
    joint_ids: Vec<JointId>,
}

impl ModelReport {
    /// The report's rows and loop bullets as display lines, plus the label
    /// column width they're padded to — shared by [`fmt::Display`] and
    /// [`ModelReport::render_with_mobility`] so a caller-supplied mobility
    /// line can align with the rest of the table.
    fn lines(&self) -> (Vec<String>, usize) {
        let rows = [
            (
                "bodies".to_string(),
                format!("{} (+ ground)", self.body_count),
            ),
            ("joints".to_string(), self.joints_summary()),
            (
                "primary coordinates".to_string(),
                format!(
                    "{} (free: {}, prescribed: {})",
                    self.primary_coordinate_count,
                    self.free_coordinate_count,
                    self.primary_coordinate_count - self.free_coordinate_count
                ),
            ),
            ("motions".to_string(), self.motion_count.to_string()),
            ("solver".to_string(), self.solver_summary()),
            ("topology".to_string(), self.topology_summary()),
        ];
        let label_width = rows.iter().map(|(label, _)| label.len()).max().unwrap_or(0);

        let mut lines = rows
            .iter()
            .map(|(label, value)| format!("{label:<label_width$}  {value}"))
            .collect::<Vec<_>>();

        if let TopologyReport::ClosedLoop { loops } = &self.topology {
            for (index, loop_report) in loops.iter().enumerate() {
                lines.push(format!("  loop {}: {loop_report}", index + 1));
            }
        }

        (lines, label_width)
    }

    /// Renders the report with a trailing `mobility` row, aligned to the
    /// same label column as the rest of the table.
    ///
    /// Mobility classification is fallible and computed elsewhere (`km
    /// check`, via `mobility_at`); this just renders an already-computed
    /// value as part of the report's layout.
    pub fn render_with_mobility(&self, mobility: impl fmt::Display) -> String {
        let (mut lines, label_width) = self.lines();
        lines.push(format!("{:<label_width$}  {mobility}", "mobility"));
        lines.join("\n")
    }
}

impl fmt::Display for ModelReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.lines().0.join("\n"))
    }
}

impl ModelReport {
    /// Builds a report from `problem`.
    ///
    /// Pure and infallible — no solving happens here. Mobility classification
    /// (which can fail) is layered on top by `km check`, not inside this report.
    pub fn build(problem: &Model) -> ModelReport {
        let mut joint_kind_counts = BTreeMap::new();
        for joint in problem.mechanism().joints().iter() {
            *joint_kind_counts.entry(joint.kind()).or_insert(0) += 1;
        }

        let topology = if problem.closure_joint_ids().is_empty() {
            TopologyReport::OpenTree
        } else {
            TopologyReport::ClosedLoop {
                loops: problem
                    .closure_joint_ids()
                    .iter()
                    .map(|&closure_joint_id| LoopReport {
                        closure_joint_id,
                        joint_ids: problem
                            .topology()
                            .loop_joint_ids(problem.mechanism(), closure_joint_id),
                    })
                    .collect(),
            }
        };

        ModelReport {
            body_count: problem.mechanism().bodies().iter().count() - 1,
            joint_count: problem.mechanism().joints().iter().count(),
            joint_kind_counts,
            primary_coordinate_count: problem.primary_coordinate_count(),
            free_coordinate_count: problem.free_primary_coordinates().len(),
            motion_count: problem.motions().iter().count(),
            solver: problem.solver(),
            topology,
        }
    }

    fn joints_summary(&self) -> String {
        if self.joint_kind_counts.is_empty() {
            return self.joint_count.to_string();
        }

        let kinds = self
            .joint_kind_counts
            .iter()
            .map(|(kind, count)| format!("{}: {count}", joint_kind_label(*kind)))
            .collect::<Vec<_>>()
            .join(", ");
        format!("{} ({kinds})", self.joint_count)
    }

    fn solver_summary(&self) -> String {
        format!(
            "t={}..{}, step {} ({} steps)",
            self.solver.start_time(),
            self.solver.end_time(),
            self.solver.step_size(),
            self.solver.times().count()
        )
    }

    fn topology_summary(&self) -> String {
        match &self.topology {
            TopologyReport::OpenTree => "open tree".to_string(),
            TopologyReport::ClosedLoop { loops } => {
                format!(
                    "closed loop ({} loop{})",
                    loops.len(),
                    if loops.len() == 1 { "" } else { "s" }
                )
            }
        }
    }
}

impl fmt::Display for LoopReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `joint_ids` ends with the closure joint itself; show the rest of
        // the tree path, then the closure joint separately.
        let tree_path = &self.joint_ids[..self.joint_ids.len() - 1];
        let joints = tree_path
            .iter()
            .map(JointId::to_string)
            .collect::<Vec<_>>()
            .join(", ");
        write!(f, "joints {joints} (closure {})", self.closure_joint_id)
    }
}

fn joint_kind_label(kind: JointKind) -> &'static str {
    match kind {
        JointKind::Spherical => "spherical",
        JointKind::Revolute => "revolute",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::yaml::parse_yaml_str;

    #[test]
    fn builds_open_tree_report() {
        let problem = parse_yaml_str(include_str!(
            "../tests/fixtures/revolute_one_body_motion.yaml"
        ))
        .unwrap()
        .into_model()
        .unwrap();

        let report = ModelReport::build(&problem);

        assert_eq!(report.body_count, 1);
        assert_eq!(report.joint_count, 1);
        assert_eq!(report.topology, TopologyReport::OpenTree);
        assert!(
            report
                .to_string()
                .contains("topology             open tree")
        );
    }

    #[test]
    fn builds_closed_loop_report_with_loop_structure() {
        let problem = parse_yaml_str(include_str!("../tests/fixtures/four_bar_linkage.yaml"))
            .unwrap()
            .into_model()
            .unwrap();

        let report = ModelReport::build(&problem);

        let TopologyReport::ClosedLoop { loops } = &report.topology else {
            panic!("expected closed-loop topology");
        };
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].closure_joint_id, JointId::new(4));
        assert_eq!(
            loops[0].joint_ids,
            vec![
                JointId::new(3),
                JointId::new(2),
                JointId::new(1),
                JointId::new(4),
            ]
        );
        assert!(
            report
                .to_string()
                .contains("loop 1: joints 3, 2, 1 (closure 4)")
        );
    }
}
