---
description: Pass 3 - inspect source-level bloat, duplication and unnecessary abstraction
agent: rust-auditor
---

Load the `rust-project-audit` skill.

Perform pass 3 of 5: source-level bloat and general Rust style audit.

Inspect the repository for concrete examples of:

- duplicated logic
- repeated mappings and conversions
- pass-through functions or layers
- avoidable boilerplate
- unnecessary allocation or cloning
- excessive indirection
- speculative traits, generics or abstractions
- dead or nearly unused code
- overly broad public visibility
- long functions, modules or impl blocks with mixed responsibilities
- inconsistent naming or error-handling patterns
- comments or helper types that make simple code harder to follow

Use Clippy and Cargo checks only as supporting evidence, not as the audit itself.
Do not apply automatic fixes.

Rank findings by impact on comprehension, correctness and change cost. Include
specific paths and symbols, plus a `Keep` section for code that is verbose but
justified. Do not edit anything.
