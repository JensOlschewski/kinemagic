---
description: Pass 2 - inspect structs, enums, traits, builders and conversions
agent: rust-auditor
---

Load the `rust-project-audit` skill.

Perform pass 2 of 5: type-system and data-object audit.

Inventory the architecturally relevant structs, enums, traits, type aliases,
newtypes, builders, configuration objects and conversions.

For each important type, determine from actual usage:

- its responsibility
- where it is created and consumed
- whether it is mutable
- whether it carries a distinct invariant or lifecycle stage
- similar or overlapping types
- conversion direction and frequency
- whether the abstraction adds semantics, validation or ergonomics

Look specifically for:

- multiple types representing the same responsibility
- wrappers or builders that add no clear value
- overly large structs
- excessive optional or boolean state
- unnecessary traits or generic parameters
- aliases that obscure rather than clarify
- repeated conversion layers
- broad derives such as `Clone` or `Default` without clear need

Do not infer redundancy from field similarity alone.

Return a compact type table followed by confirmed findings, suspected findings
and a `Keep` section. Do not edit anything.
