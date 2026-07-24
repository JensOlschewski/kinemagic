---
description: Pass 1 - map the repository without judging or changing it
agent: rust-auditor
---

Load the `rust-project-audit` skill.

Perform pass 1 of 5: repository mapping.

Inspect the complete Rust project at a high level before making architectural
judgements. Read the manifests, source tree, module declarations, binaries,
library entry points, tests and examples.

Report:

1. crates, targets and entry points
2. module tree and apparent responsibilities
3. important execution paths
4. key structs, enums, traits and builders
5. current implementation maturity: implemented, partial or placeholder
6. unusually large, fragmented or unclear areas
7. questions that cannot yet be answered from the code

Keep this pass primarily descriptive. Initial concerns may be noted, but do not
propose a redesign and do not edit anything.
