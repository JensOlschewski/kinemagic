# Kinemagic

A simple CLI for modeling and solving multibody mechanisms.

## Quick start

```bash
cargo install --path .
km check examples/spherical_one_body_motion.yaml
km solve examples/spherical_one_body_motion.yaml
km solve examples/spherical_one_body_motion.yaml --view
```

## Input

Examples:

- [2D one-body motion](examples/spherical_one_body_motion.yaml)
- [2D three-body chain](examples/spherical_three_body_chain.yaml)
- [3D spherical plate](examples/spherical_plate_3d_motion.yaml)

![Animated terminal viewer](docs/assets/viewer.gif)


M1 currently supports:

- spherical joints
- ground-rooted open trees and closed loops
- Euler Z-X-Z orientations in degrees
- time-dependent joint-coordinate motions
- terminal reference, after-solve, and live views

## Output

`km solve` writes solved body poses to stdout. Use `--view` for after-solve
playback or `--view=live` to view frames as they solve.

## Development

```bash
cargo test
cargo clippy --all-targets -- -D warnings
```
