# Kinemagic

A simple CLI for modeling and solving multibody mechanisms.

## Quick start

```bash
cargo install --path .
km check examples/spherical_one_body_motion.yaml
km solve examples/spherical_one_body_motion.yaml
```

## Input

See [canonical example](examples/spherical_one_body_motion.yaml).

M1 currently supports:

- spherical joints
- ground-rooted trees
- Euler Z-X-Z orientations in degrees
- optional joint-coordinate motions

## Output

`km solve` writes solved body poses to stdout.

## Development

```bash
cargo test
cargo clippy --all-targets -- -D warnings
```
