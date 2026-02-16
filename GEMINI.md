# Neurorobotics Workspace (NRT_WS) Guidelines

> **Architectural Reference**: The detailed system architecture, including class hierarchies, data flow, and API usage, is documented in `ARCHITECTURE.md`. **You are expected to read `ARCHITECTURE.md`** to understand the system design before modifying core components.

## Commit Message Conventions

To maintain a clean and informative project history, all commits must adhere to the following strict guidelines.

### Structure

The commit message should be structured with a subject line, followed by `Problem`, `Solution`, and `Note` sections.

```text
<Subject Line>

Problem
=======
<Brief description of the issue or motivation (max 3 lines)>

Solution
========
<Detailed list of changes>

Note
====
<Additional context, breaking changes, or deletions>
```

### Style Rules

1.  **Subject Line**:
    *   Imperative mood (e.g., "Fix bug" not "Fixed bug").
    *   Capitalized start, no trailing period.
    *   **Max 50 characters**.
    *   Blank line after the subject.

2.  **Content**:
    *   **Wrap at 72 characters**.
    *   **Method References**: Must include the class scope (e.g., `LifecycleControllerBase::SyncCallChangeState`).
    *   **File Paths**: Must use the full relative path from the repo root (e.g., `src/controllers/include/controllers/common/controller.h`).

3.  **Sections**:
    *   **Problem**: *What* is broken or missing and *why* it needs fixing.
    *   **Solution**: *What* specific changes were made to address the problem.
    *   **Note**: Deletions, side effects, or special instructions.

## Basalt Codebase Insights

When working with the `basalt` component of the workspace, observe the following architectural patterns and idiosyncrasies:

*   **Thread Safety & Queues**: The system relies heavily on `tbb::concurrent_bounded_queue` for communication between the frontend (optical flow) and backend (estimator). Ensure queues are properly connected and drained.
*   **Factory Pattern**: Major components (`OpticalFlowBase`, `VioEstimatorBase`) are instantiated via factories (`OpticalFlowFactory`, `VioEstimatorFactory`) based on `VioConfig`. Avoid direct instantiation of concrete classes in application code.
*   **Configuration**: The `basalt::VioConfig` struct ([`include/basalt/utils/vio_config.h`](include/basalt/utils/vio_config.h)) is the central source of truth for runtime parameters. It uses `cereal` for serialization, so adding parameters requires modifying `src/utils/vio_config.cpp` to register them via `ar(CEREAL_NVP(...))`.
*   **Serialization (Cereal)**:
    *   **JSON**: Used for config and small result summaries. Always use `CEREAL_NVP` to name your fields in JSON.
    *   **Binary**: Used for heavy data (caches, marginalization data).
    *   **Custom Types**: Support for Eigen and Sophus types is built-in via headers in `basalt/serialization/`.
*   **Math Types**:
    *   **Sophus**: Used extensively for Lie Group operations (`SE3`, `SO3`, `SE2`).
    *   **Eigen**: Used for linear algebra. Note the use of `Eigen::aligned_vector` and `Eigen::aligned_map` to handle alignment requirements.
    *   **Templating**: Core classes are often templated on `Scalar` (float vs. double) and `Pattern`.
*   **Data Types**:
    *   `OpticalFlowInput`: Contains raw image data.
    *   `OpticalFlowResult`: Contains tracked keypoints.
    *   `ImuData`: Contains accelerometer and gyroscope readings.
    *   `PoseVelBiasState`: Represents the estimated state.

> **Note on Data Structures**: Any newly added or significantly modified data structures must be documented in the "Core Data Structures" section of `ARCHITECTURE.md`.

## Helper Macros & Utilities

When writing code, prefer the project's established macros for assertions and logging over standard `assert()` or `std::cerr`:

*   **Assertions**: Use `BASALT_ASSERT(expr)` or `BASALT_ASSERT_STREAM(expr, msg)` to verify assumptions.
*   **Logging**: Use `BASALT_LOG_FATAL_STREAM("msg " << val)` for critical errors.
*   **Formatting**: Use the `_format` literal for string formatting: `"Val: {}"_format(x)`.
*   **Unused Variables**: Use `UNUSED(var)` to silence compiler warnings.

Refer to the "Helper Macros & Utilities" section in `ARCHITECTURE.md` for the full list.