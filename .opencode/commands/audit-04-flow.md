---
description: Pass 4 - trace real control flow, data flow and module responsibilities
agent: rust-auditor
---

Load the `rust-project-audit` skill.

Perform pass 4 of 5: architecture and data-flow audit.

Choose the project's main user-visible or solver workflows and trace them from
entry point to final result. Follow the real code rather than inferred module
names.

For each workflow show:

- entry point
- main calls
- data types passed between stages
- transformations and conversions
- validation and error handling
- ownership and mutation
- final consumer or output

Identify only evidence-backed concerns such as:

- unclear responsibility boundaries
- logic in surprising modules
- repeated state or multiple sources of truth
- unnecessary back-and-forth conversion
- hard-to-follow ownership
- dependency cycles or awkward coupling
- abstractions that obscure the main workflow

Also identify boundaries that are useful and should be kept.

Do not design the replacement architecture. Do not edit anything.
