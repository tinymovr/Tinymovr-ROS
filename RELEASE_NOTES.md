# Tinymovr ROS 3.0.0 (firmware 3.0.x)

This release targets Tinymovr firmware **3.0.x**. The protocol bindings under
`include/tinymovr/` and `src/tinymovr/` were regenerated from
`tinymovr_3_0_x.yaml` via [Avlos](https://github.com/tinymovr/avlos).

## Changes

- Added `MA600` to the External SPI sensor type enum
  (`SENSORS_SETUP_EXTERNAL_SPI_TYPE_MA600 = 3`).
- Updated `avlos_proto_hash` to match the new spec
  (`3999954334` -> `3786962419`).
- Bumped package version to `3.0.0` in `package.xml`.
- Added `AGENTS.md` documenting Avlos-based protocol regeneration and the
  release flow.

## Compatibility

- The protocol change is additive (a new enum value); existing user code does
  not need to change.
- The Tinymovr device firmware must be the matching 3.0.x release, since the
  protocol hash has changed.

## Installation

Follow instructions on the [GitHub repository](https://github.com/tinymovr/Tinymovr-ROS)
to install the latest version.
