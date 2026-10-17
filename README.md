# Kinemagic

A simple CLI for modeling and solving multibody mechanisms.

## Quick start

```bash
cargo install --path .
km check examples/revolute_one_body_motion.yaml
km solve examples/revolute_one_body_motion.yaml
km solve examples/revolute_one_body_motion.yaml --view
```

## Input

Examples:

- [2D three-body chain](examples/spherical_three_body_chain.yaml)
- [3D spherical plate](examples/spherical_plate_3d_motion.yaml)
- [Revolute hinge motion](examples/revolute_one_body_motion.yaml)
- [Four-bar closed loop (revolute + spherical)](examples/four_bar_linkage.yaml)

![Animated terminal viewer](docs/assets/viewer.gif)

Currently supports:

- spherical and revolute joints
- ground-rooted open trees and closed loops
- Euler Z-X-Z orientations in degrees
- time-dependent joint-coordinate motions
- terminal reference and after-solve views

## Output

`km solve` writes solved body poses to stdout. Use `--view` for after-solve
playback.

## Development

```bash
cargo test
cargo clippy --all-targets -- -D warnings
```
