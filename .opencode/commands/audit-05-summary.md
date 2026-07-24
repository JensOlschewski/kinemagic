---
description: Pass 5 - consolidate and prioritize the complete project audit
agent: rust-auditor
---

Load the `rust-project-audit` skill.

Perform pass 5 of 5: consolidated assessment.

Use the previous four passes from this conversation. Re-check the repository
where findings conflict or lack evidence. Do not merely concatenate earlier
reports.

Produce:

## Current state

A concise description of how the project is structured and how complete it is.

## Keep

The strongest existing decisions, abstractions and boundaries.

## Confirmed findings

Only findings supported by definitions and actual usage. For each include path,
symbols, evidence, impact, confidence and smallest sensible change.

## Suspected findings

Items that may be bloat or redundancy but require domain knowledge, runtime
evidence or design intent before changing.

## Top five refactoring candidates

Rank no more than five candidates by:

- expected simplification
- correctness risk
- effort
- regression risk

## Suggested sequence

A small-step investigation or refactoring order. Do not provide implementation
patches and do not propose a broad rewrite.

## Verification gaps

Tests, examples or documentation that would be needed before changing the
highest-risk areas.

Remain read-only.
