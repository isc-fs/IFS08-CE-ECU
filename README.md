# IFS08 - CE-ECU

This branch contains an initial firmware base migrated from
`jbmata/VCU08-NSIL-Software`.

It is intentionally a first import: naming, directory layout, and some
project files still reflect the source repository and will be cleaned up
incrementally as the CE-ECU codebase is organized.

## What This Base Does

- Provides a firmware base for STM32H733.
- Uses FreeRTOS for task scheduling and shared-state management.
- Implements control logic and startup state handling.
- Handles CAN-FD communication with inverter, battery/ACU, and dashboard.
- Includes telemetry, diagnostics, and test infrastructure.
- Ships with unit tests and SIL integration tests that can run on PC.

## Repository Layout

- `Core/`: application sources and headers.
- `Drivers/`: STM32 HAL and CMSIS dependencies.
- `Middlewares/`: FreeRTOS sources.
- `tests/unit/`: unit tests.
- `tests/sil/`: SIL harness and integration scenarios.
- `ECU_LOGIC_REPORT.md`: control-logic overview.
- `INTEGRATION_TESTS_EXPLAINED.md`: SIL test documentation.

## Migration Notes

- This is the initial imported version of the project.
- Cleanup and adaptation to the final CE-ECU naming are still pending.
- The STM32Cube project file keeps the original name
  `ECU08 NSIL.ioc` for now.

## Development Workflow

Branching and issue-tracking conventions for this repository are documented in
`docs/REPOSITORY_WORKFLOW.md`.
