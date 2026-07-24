---
name: rust-project-audit
description: Perform neutral, evidence-based, read-only audits of an existing Rust repository, focusing on project structure, type usage, unnecessary complexity, duplication, code bloat and real data flow.
compatibility: opencode
metadata:
  language: rust
  workflow: audit
---

# Rust Project Audit

## Purpose

Understand an existing Rust project before recommending changes.

Do not assume that the current structure is wrong. Do not impose a target
architecture. Discover the actual responsibilities and usage first.

## Working rules

- Remain read-only.
- Inspect the whole relevant repository, not only a few prominent files.
- Follow symbols through definitions, impl blocks, constructors, conversions,
  call sites and tests.
- Use LSP definitions and references when available.
- Use repository search to validate claims.
- Distinguish source-code bloat from binary-size bloat.
- Do not call something redundant only because two types have similar fields.
- Do not call something over-engineered only because it uses traits, generics or
  builders. Verify whether the abstraction has a real purpose.
- Do not recommend merging types until their lifecycle, invariants, ownership and
  consumers are understood.
- Do not recommend a rewrite during an audit pass.
- Do not install tools or modify dependencies.
- Never run `cargo fix`.

## Optional non-destructive checks

Use only when useful and after inspecting `Cargo.toml`:

```bash
cargo metadata --no-deps
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo clippy --workspace --all-targets
cargo test --workspace
cargo tree --duplicates
```

Adapt commands to the repository. Do not assume every project is a workspace or
that every feature can be enabled simultaneously.

## Evidence standard

For each significant finding include:

- file path
- symbol or module
- observation
- evidence from usage
- why it may matter
- confidence: high, medium or low
- smallest reasonable follow-up investigation or change

Avoid vague findings such as "this could be cleaner."

## Areas to inspect

### Project structure

- crates, binaries and libraries
- module tree
- entry points
- public surface
- tests, examples and support code
- unusually large or fragmented modules
- unclear module responsibilities

### Types

- structs, enums and unions
- traits and implementations
- type aliases and newtypes
- builders and configuration types
- conversion chains
- repeated representations
- broad `Clone`, `Copy` or `Default`
- large structs with unrelated responsibilities
- many `Option` or boolean fields encoding state
- wrappers that add no observable semantics
- traits or generics with no demonstrated benefit

### Source-level bloat

- repeated logic
- repeated mapping or conversion code
- pass-through functions and wrappers
- boilerplate implementations
- avoidable cloning or allocation
- unnecessary indirection
- speculative abstraction
- dead or nearly dead code
- overly broad visibility
- long functions or impl blocks with mixed responsibilities
- comments that compensate for unclear code rather than explain intent

### Architecture and flow

- actual control and data flow
- where values are created, transformed and consumed
- ownership and mutation
- duplicated state or multiple sources of truth
- responsibility leakage between modules
- cycles or awkward dependency directions
- error handling and validation placement
- boundaries that are useful and should remain

## Reporting

Be compact but concrete.

Do not produce a long catalogue of every minor style preference. Prioritize
issues that affect comprehension, correctness, change cost or maintainability.

Always include a section named `Keep` for structures and abstractions that appear
justified and should not be removed casually.
