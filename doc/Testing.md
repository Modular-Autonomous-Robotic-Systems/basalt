# Basalt SLAM Testing Framework

## 1. Introduction

This document provides a comprehensive reference for the testing infrastructure of the Basalt SLAM system. An exhaustive overview of all existing tests and a step-by-step guide for adding new tests is provided. The Basalt testing framework uses Google Test (gtest) as its test runner, vendored directly into the repository rather than relying on a system installation. Tests are divided into two categories, as per standard practises:
- **Unit tests**: Validate individual mathematical operations, Jacobians, decompositions, and data structures in isolation using synthetically generated data. These are gtest executables defined in `test/CMakeLists.txt`.
- **Integration tests**: End-to-end pipeline validation using recorded datasets (e.g., EuRoC). These are standalone executables defined in the root `CMakeLists.txt` (e.g., `basalt_vio`, `basalt_mapper`, and the new `basalt_slam`).

A detailed case study of added tests for `class LocalMapper` and other classes required to run local mapping is discussed in detail to explain the process of adding new tests to Basalt. The testing framework in Basalt is distributed according the following directory structure:
```
basalt/
├── test/
│   ├── CMakeLists.txt              ← Defines unit test executables
│   └── src/
│       ├── test_linearization.cpp  ← Bundle adjustment linearisation consistency
│       ├── test_vio.cpp            ← IMU preintegration, nullspace, Jacobians
│       ├── test_nfr.cpp            ← NFR factor Jacobians (RelPose, RollPitch, etc.)
│       ├── test_qr.cpp             ← QR vs LLT decomposition, Schur complement
│       ├── test_patch.cpp          ← Optical flow patch SE2 Jacobians
│       ├── test_spline_opt.cpp     ← Spline optimisation with IMU + pose
│       └── test_local_mapper.cpp   ← [NEW] LocalMapper unit tests
├── thirdparty/basalt-headers/test/
│   ├── CMakeLists.txt              ← Adds gtest subdirectory + basalt-headers tests
│   ├── include/test_utils.h        ← Shared test utilities (Jacobian checker)
│   └── googletest/                 ← Vendored Google Test source
│       └── googletest/
│           └── include/gtest/gtest.h
├── src/
│   ├── vio.cpp                     ← Reference VIO integration test (with Pangolin GUI)
│   ├── mapper.cpp                  ← Reference offline mapper integration test
│   └── basalt_slam.cpp             ← [NEW] SLAM integration test executable (no GUI)
├── data/
│   ├── euroc_config.json           ← EuRoC VIO configuration
│   ├── euroc_ds_calib.json         ← EuRoC double-sphere calibration
│   └── ...                         ← Other configs and calibration files
└── CMakeLists.txt                  ← Root build; defines library + executables
```

`CMakeLists.txt`** includes the `gtest` installation used from `thirdparty/basalt-headers/test` directory and the test executables from `test`.
```cmake
enable_testing()
add_subdirectory(thirdparty/basalt-headers/test)  # Brings in gtest
add_subdirectory(test)                             # Brings in basalt unit tests
```

The basalt-headers test CMakeLists.txt at `thirdparty/basat-headers/test/CMakeLists.txt` conditionally adds the vendored googletest:
```cmake
if(NOT TARGET gtest_main)
  add_subdirectory(googletest ...)
endif()
```

In `test/CMakeLists.txt`** each test executable links `gtest`, `gtest_main`, and `basalt`:
```cmake
add_executable(test_nfr src/test_nfr.cpp)
target_link_libraries(test_nfr gtest gtest_main basalt)
```

The following table lists down all the unit tests implemented within basalt testing framework:

| Executable | Source | Suite | # Tests | Domain |
|---|---|---|---|---|
| `test_linearization` | `test/src/test_linearization.cpp` | `LinearizationTestSuite` | 4 | BA linearisation consistency (ABS_QR, ABS_SC, REL_SC) |
| `test_vio` | `test/src/test_vio.cpp` | `VioTestSuite` | 4 | IMU nullspace, relative pose, point linearisation |
| `test_nfr` | `test/src/test_nfr.cpp` | `PreIntegrationTestSuite` | 4 | NFR factor Jacobians (relPose, absPosition, yaw, rollPitch) |
| `test_qr` | `test/src/test_qr.cpp` | `QRTestSuite` | 3 | QR/LLT decomposition, rank-deficient Schur complement |
| `test_patch` | `test/src/test_patch.cpp` | `Patch` | 2 | Optical flow patch gradient and SE2 Jacobians |
| `test_spline_opt` | `test/src/test_spline_opt.cpp` | `SplineOpt` | 1 | Spline optimisation with simulated IMU/pose |
| `test_local_mapper` | `test/src/test_local_mapper.cpp` | `LocalMapperTestSuite` | 7 | UnionFind, TrackBuilder, HashBowStl, LocalMapper |

The following table lists down relevant integration tests implemented within the basalt testing framework:

| Executable | Source | Purpose |
|---|---|---|
| `basalt_vio` | `src/vio.cpp` | Offline VIO with Pangolin GUI |
| `basalt_mapper` | `src/mapper.cpp` | Offline NFR mapper with GUI |
| `basalt_slam` | `src/basalt_slam.cpp` | [NEW] Full SLAM pipeline (VIO + LocalMapper), no GUI |


The implemented executables perform the following specific tests:
1. **`test_linearization` — Bundle Adjustment Linearisation Consistency**: Verifies that three different linearisation strategies (ABS_QR, ABS_SC, REL_SC) produce identical Hessian matrices, gradient vectors, and error values for the same synthetic BA problem. Tests both with and without marginalisation priors, and validates backsubstitution correctness.
   - `VoNoMargLinearizationTest` — Compares ABS_QR, ABS_SC, and REL_SC linearisations on a synthetic VO problem without marginalisation; asserts H, b, and error agree within 1e-8.
   - `VoMargLinearizationTest` — Same three-strategy comparison but with marginalisation priors applied to the first two frames.
   - `VoMargBacksubstituteTest` — Extends the marginalised test by solving the system, backsubstituting into all three linearisations, and verifying that landmark updates and final errors agree.
   - `VoMargSqrtLinearizationTest` — Extracts the square-root form (Q2^T·Jp, Q2^T·r) from each strategy and verifies J^T·J ≈ H and J^T·r ≈ b.

2. **`test_vio` — VIO IMU Preintegration and Nullspace Validation**: Validates that: (a) the IMU preintegration nullspace correctly spans 7 DOF (6-DOF pose + scale), (b) the linearised quadratic error model agrees with the true non-linear error for small perturbations, (c) relative pose Jacobians match numerical finite differences, and (d) point linearisation Jacobians are correct.
   - `ImuNullspace2Test` — Two-state IMU preintegration: checks the Hessian nullspace spans the expected DOF and that the quadratic approximation matches the non-linear error for small perturbations.
   - `ImuNullspace3Test` — Three-state (two-segment) IMU preintegration: validates the accumulated Hessian's nullspace over a longer trajectory with two IMU segments.
   - `RelPoseTest` — Tests analytical Jacobians of `computeRelPose` w.r.t. host and target poses against numerical finite differences via `test_jacobian()`.
   - `LinearizePointsTest` — Tests the Jacobians of `linearizePoint` w.r.t. relative pose (`d_res_d_xi`) and point parameters (`d_res_d_p`) against numerical finite differences.

3. **`test_nfr` — Non-Linear Factor Recovery Jacobians**: Tests the analytical Jacobians of the four NFR residual functions (`relPoseError`, `absPositionError`, `yawError`, `rollPitchError`) against numerical central finite differences via `test_jacobian()`.
   - `RelPoseTest` — Validates `relPoseError` Jacobians w.r.t. both `T_w_i` and `T_w_j` against numerical finite differences.
   - `AbsPositionTest` — Validates `absPositionError` Jacobian w.r.t. `T_w_i` against numerical finite differences.
   - `YawTest` — Validates `yawError` Jacobian w.r.t. `T_w_i` against numerical finite differences.
   - `RollPitchTest` — Validates `rollPitchError` Jacobian w.r.t. `T_w_i` against numerical finite differences.

4. **`test_qr` — QR Decomposition and Schur Complement**: Compares QR vs LLT factorisations on random matrices (including rank-deficient cases). Validates that the three `MargHelper` strategies (`SqToSq`, `SqToSqrt`, `SqrtToSqrt`) produce consistent solutions.
   - `QRvsLLT` — Compares Householder QR and Cholesky (LLT) factorisations on a random full-rank 10×6 matrix; prints R and L^T for visual inspection.
   - `QRvsLLTRankDef` — Same QR-vs-LLT comparison on a rank-deficient matrix (duplicate column) to demonstrate factorisation behaviour in degenerate cases.
   - `RankDefLeastSquares` — Solves a rank-deficient least-squares problem via `MargHelper::SqToSq`, `SqToSqrt`, and `SqrtToSqrt`, asserting all three produce identical solutions.

5. **`test_patch` — Optical Flow Patch Tracking**: Verifies image interpolation gradients and the Jacobian of the patch residual with respect to SE2 transformations, using a synthetic smooth function.
   - `ImageInterpolateGrad` — Verifies the analytical gradient of a smooth synthetic interpolation function against numerical finite differences.
   - `PatchSe2Jac` — Validates the Jacobian of the 52-element patch residual w.r.t. SE2 transformations, and checks that `setDataJacSe2` and `setData` produce consistent values.

6. **`test_spline_opt` — Spline Optimisation**: Creates a random SE3 spline trajectory, generates synthetic IMU and pose measurements, optimises the spline, and verifies recovery of calibration parameters (accel bias, gyro bias, gravity vector).
   - `SplineOptTest` — Generates a random SE3 spline with synthetic IMU/pose measurements, runs 10 optimisation iterations, and asserts recovery of accel bias, gyro bias, gravity, and full spline trajectory (position, orientation, acceleration, angular velocity).

7. **`test_local_mapper` — LocalMapper Infrastructure Unit Tests**
   - `UnionFindAddIndexTest` — Verifies `AddIndex()` returns correct indices and `Find()` resolves correctly.
   - `UnionFindInvalidateRootTest` — Verifies `InvalidateRoot()` propagates `InvalidIndex()` through path compression.
   - `TrackBuilderAddNewMatchesBrandNew` — Brand-new match creates a new track with a stable TrackId.
   - `TrackBuilderAddNewMatchesAppend` — New match extends an existing track without changing its TrackId.
   - `TrackBuilderAddNewMatchesMergeOneLandmark` — Merge where one track has an lmdb landmark: the landmark-bearing track survives.
   - `TrackBuilderAddNewMatchesMergeBothLandmarks` — Merge where both tracks have landmarks: the larger track survives; the smaller's TrackId appears in `retired_ids`.
   - `TrackBuilderCompactTree` — After partial invalidation, `CompactTree()` produces a dense DSU with identical TrackId-to-feature mappings.
   - `HashBowStlRemoveKeyframes` — After adding 3 KFs and removing 1, queries return only the remaining 2.

8. **`basalt_slam` — Full SLAM Integration Test**
   Feeds a recorded EuRoC dataset through `Controller` with `useProducerConsumerArchitecture=false`; verifies that: (a) the local mapper thread runs, (b) `frame_poses` in the local mapper is non-empty after processing, (c) the local map size stays bounded by `mpMaxLocalMapSize`.

The remainder of this document is organised as follows:
- **Section 2** provides an overview of `gtest` and the API used in the basalt testing framework
- **Section 3** provides a detailed deep-dive into each existing test executable.
- **Section 4** plans and implements all new tests for the LocalMapper module.
- **Section 5** is a reference catalogue of classes, methods, and data structures used throughout the test suite.

## 2. Google Test (gtest)

Google Test (gtest) is Google's xUnit-style C++ testing framework. It provides a structured way to write, organise, and run unit tests for C++ code. The framework is built around a small set of core concepts:

- **Test case**: A single `TEST()` function that exercises one behaviour and checks results with assertions.
- **Test suite**: A logical grouping of related test cases that share the first argument to `TEST(Suite, Name)`. For example, all four NFR Jacobian tests belong to the suite `PreIntegrationTestSuite`.
- **Assertion**: A check inside a test body. Assertions come in two flavours — non-fatal (`EXPECT_*`) and fatal (`ASSERT_*`).
- **Test runner**: The `gtest_main` library provides a default `main()` entry point that discovers all registered tests and executes them via `RUN_ALL_TESTS()`.

In Basalt, gtest is **vendored** (committed directly into the repository) at `thirdparty/basalt-headers/test/googletest/` rather than relying on a system-wide installation. This guarantees a consistent framework version across all build environments and removes an external dependency. The `gtest_main` library provides a default `main()` function, so individual test source files only need to define `TEST()` bodies — no custom `main()` is required.

**Basalt API conventions and assumptions:**
- All tests use the plain `TEST(Suite, Name)` macro. The fixture-based `TEST_F(Fixture, Name)` macro is available but is **not used** in the Basalt test suite.
- Parameterised tests (`TEST_P`) and death tests (`EXPECT_DEATH`) are not used.
- Each test source file typically belongs to a single suite (e.g., `test_nfr.cpp` → `PreIntegrationTestSuite`).
- Test bodies are self-contained: they generate synthetic data, invoke the function under test, and assert correctness — no shared state leaks between tests.

### Key Macros

| Macro | Purpose | Example |
|---|---|---|
| `TEST(Suite, Name)` | Define a standalone test case | `TEST(QRTestSuite, QRvsLLT) { ... }` |
| `TEST_F(Fixture, Name)` | Define a test using a fixture class | Not used in basalt's test suite |
| `EXPECT_TRUE(expr)` | Non-fatal boolean assertion | `EXPECT_TRUE(A.isApprox(B));` |
| `EXPECT_FALSE(expr)` | Non-fatal false assertion | `EXPECT_FALSE(result.empty());` |
| `EXPECT_EQ(a, b)` | Non-fatal equality | `EXPECT_EQ(uf.GetNumNodes(), 5);` |
| `EXPECT_LE(a, b)` | Non-fatal less-or-equal | `EXPECT_LE(error, 1e-6);` |
| `EXPECT_NEAR(a, b, abs_error)` | Non-fatal approximate equality | `EXPECT_NEAR(val, 0.0, 1e-10);` |
| `ASSERT_TRUE(expr)` | Fatal boolean assertion (stops test on failure) | `ASSERT_TRUE(kpt.isApprox(...));` |
| `ASSERT_EQ(a, b)` | Fatal equality | `ASSERT_EQ(tracks.size(), 3);` |

#### `TEST()` and `TEST_F()`

The `TEST(Suite, Name)` macro is the primary way to define a test case. It takes two arguments: the **suite name** (a logical group) and the **test name** (identifying the specific behaviour under test). gtest automatically registers each `TEST()` at compile-time via static initialisation — there is no manual registration step. At runtime, `RUN_ALL_TESTS()` iterates over all registered tests.

```cpp
// From test/src/test_nfr.cpp — defines test "RelPoseTest" in suite "PreIntegrationTestSuite"
TEST(PreIntegrationTestSuite, RelPoseTest) {
  Sophus::SE3d T_w_i = Sophus::se3_expd(Sophus::Vector6d::Random());
  Sophus::SE3d T_w_j = Sophus::se3_expd(Sophus::Vector6d::Random());
  // ... compute residual, Jacobian, call test_jacobian() ...
}
```

`TEST_F(Fixture, Name)` allows a test to inherit from a *fixture class* (a subclass of `::testing::Test`) that provides shared `SetUp()` and `TearDown()` logic. This is useful when multiple tests need identical initialisation. Basalt does not use `TEST_F` — each test constructs its own data inline.

#### `EXPECT_*` Non-Fatal Assertions

`EXPECT_*` assertions record a failure but **allow the test body to continue executing**. This means a single test run can report multiple failures, which is useful for diagnosing several issues at once. The Basalt test suite uses `EXPECT_*` extensively:

```cpp
// From thirdparty/basalt-headers/test/include/test_utils.h — inside test_jacobian()
EXPECT_TRUE(Ja.allFinite()) << name << ": Ja not finite\n " << Ja;
EXPECT_TRUE(Jn.isApprox(Ja, max_norm))
    << name << ": Ja not equal to Jn (diff norm:" << (Jn - Ja).norm() << ")";

// From test/src/test_local_mapper.cpp
EXPECT_EQ(uf.GetNumNodes(), 5);
EXPECT_EQ(uf.Find(0), uf.Find(1));
EXPECT_NE(uf.Find(0), uf.Find(2));
```

The full family includes `EXPECT_TRUE`, `EXPECT_FALSE`, `EXPECT_EQ`, `EXPECT_NE`, `EXPECT_LT`, `EXPECT_LE`, `EXPECT_GT`, `EXPECT_GE`, `EXPECT_NEAR`, `EXPECT_FLOAT_EQ`, and `EXPECT_DOUBLE_EQ`.

#### `ASSERT_*` Fatal Assertions

`ASSERT_*` assertions **abort the current test immediately** upon failure. Use `ASSERT_*` when subsequent code depends on the checked condition being true — for example, verifying a container's size before indexing into it:

```cpp
// From test/src/test_local_mapper.cpp — must confirm size before accessing elements
ASSERT_EQ(uf.AddIndex(), 0);
ASSERT_EQ(uf.AddIndex(), 1);
ASSERT_EQ(uf.GetNumNodes(), 5);

// From test/src/test_linearization.cpp — verify container is non-empty before use
ASSERT_TRUE(kpt.isApprox(...));
```

**Design rule:** Default to `EXPECT_*`. Only use `ASSERT_*` when a failure would make the remaining test code meaningless or would cause undefined behaviour (e.g., null-pointer dereference, out-of-bounds access).

#### Custom Failure Messages

Both `EXPECT_*` and `ASSERT_*` support streaming additional diagnostic information with `<<`:
```cpp
EXPECT_TRUE(A.isApprox(B, 1e-6))
    << "Matrices differ, norm: " << (A - B).norm();
```
This message is only printed when the assertion fails, providing context to aid debugging.

### Test Lifecycle

Understanding the test lifecycle requires familiarity with the following concepts:

- **Test suite**: The first argument to `TEST(Suite, Name)` — a logical grouping. gtest iterates over suites, then over the tests within each suite.
- **Test case**: A single `TEST()` function. Each test case is an independent unit of execution.
- **Test body**: The `{ ... }` block inside `TEST()`. This is where setup, invocation, and assertions happen.
- **Test fixture**: A class derived from `::testing::Test` that provides shared `SetUp()`/`TearDown()` logic. Not used in Basalt — each test constructs its own data inline, so `SetUp()` and `TearDown()` are no-ops.

The lifecycle of a gtest run proceeds as follows:

1. **Entry point**: The `gtest_main` library provides a `main()` function that calls `::testing::InitGoogleTest(&argc, argv)` to parse any command-line flags (e.g., `--gtest_filter`), then invokes `RUN_ALL_TESTS()`.
2. **Test discovery**: `RUN_ALL_TESTS()` iterates over all test suites registered at compile time. Within each suite, it iterates over the individual test cases.
3. **Test instantiation**: For each `TEST(Suite, Name)`, gtest constructs a fresh test object. For plain `TEST()` (as used in Basalt), the object is an anonymous subclass of `::testing::Test` with `SetUp()` and `TearDown()` as no-ops.
4. **Test execution**: gtest calls `SetUp()` → runs the test body → calls `TearDown()`. Assertions within the body either record failures (`EXPECT_*`) or abort the test immediately (`ASSERT_*`).
5. **Result reporting**: After all tests complete, gtest prints a summary to stdout:
   ```
   [==========] N tests from M test suites ran.
   [  PASSED  ] P tests.
   [  FAILED  ] F tests, listed below:
   ```
6. **Exit code**: `RUN_ALL_TESTS()` returns 0 if all tests passed, 1 otherwise. CTest and CI systems use this exit code to determine success or failure.

#### Lifecycle Walkthrough: `TEST(PreIntegrationTestSuite, RelPoseTest)`

To make the lifecycle concrete, consider the `RelPoseTest` from `test/src/test_nfr.cpp`:

1. **Discovery**: At CMake configure time, `gtest_add_tests(TARGET test_nfr AUTO)` parses `test_nfr.cpp` and finds `TEST(PreIntegrationTestSuite, RelPoseTest)`. CTest registers it as a test named `PreIntegrationTestSuite.RelPoseTest`.
2. **Execution begins**: When `./test/test_nfr` runs, `gtest_main` calls `RUN_ALL_TESTS()`. gtest finds the `PreIntegrationTestSuite` suite with its four tests, including `RelPoseTest`.
3. **Test body runs**:
   - Two random SE3 poses `T_w_i` and `T_w_j` are generated via `Sophus::se3_expd(Vector6d::Random())`.
   - A measured relative pose `T_i_j` is computed from the two poses with a small perturbation.
   - `basalt::relPoseError(T_i_j, T_w_i, T_w_j, &d_res_d_T_w_i, &d_res_d_T_w_j)` computes the residual and its **analytical Jacobians**.
   - `test_jacobian("d_res_d_T_w_i", d_res_d_T_w_i, lambda, x0)` is called with a lambda that perturbs `T_w_i` and recomputes the residual. The helper internally computes the **numerical Jacobian** via central finite differences and compares it against the analytical one.
   - Inside `test_jacobian()`, `EXPECT_TRUE(Ja.allFinite())` and `EXPECT_TRUE(Jn.isApprox(Ja, 1e-3))` fire. If the analytical and numerical Jacobians agree within tolerance, the assertions pass.
4. **Result**: gtest prints `[       OK ] PreIntegrationTestSuite.RelPoseTest` if all assertions passed, or `[  FAILED  ]` with diagnostic messages if any failed.

### Helpers

The shared test utility header `thirdparty/basalt-headers/test/include/test_utils.h` provides two reusable components used across the test suite. No other reusable cross-file helpers exist — functions like `get_vo_estimator()` in `test_linearization.cpp` are file-local and specific to their respective test.

#### `test_jacobian()`

This template function is the backbone of all Jacobian-correctness tests in Basalt. It validates an analytically computed Jacobian against a numerically computed one using central finite differences.

```cpp
template <typename Derived1, typename Derived2, typename F>
void test_jacobian(
    const std::string &name,
    const Eigen::MatrixBase<Derived1> &Ja,  // Analytical Jacobian
    F func,                                  // f: VecX → VecX
    const Eigen::MatrixBase<Derived2> &x0,  // Evaluation point
    double eps = TestConstants<Scalar>::epsilon,      // Finite-difference step (default 1e-8 for double)
    double max_norm = TestConstants<Scalar>::max_norm); // Tolerance (default 1e-3 for double)
```

**Algorithm — central finite differences:**

For each column $i$ of the numerical Jacobian $J_n$:

$$J_n[:, i] = \frac{f(x_0 + \varepsilon \, e_i) - f(x_0 - \varepsilon \, e_i)}{2\varepsilon}$$

where $e_i$ is the $i$-th standard basis vector and $\varepsilon$ is the finite-difference step size.

**Validation logic:**

The function performs three checks:
1. **Finiteness**: `EXPECT_TRUE(Ja.allFinite())` and `EXPECT_TRUE(Jn.allFinite())` — ensures neither matrix contains NaN or infinity.
2. **Near-zero case**: If both $J_a$ and $J_n$ are near zero (determined by `isZero(max_norm)`), the function checks that their difference is also near zero.
3. **General case**: Otherwise, it uses `Jn.isApprox(Ja, max_norm)` to compare the two matrices, which checks that the relative error is within the tolerance.

If any check fails, the assertion prints the matrix name, norms, and full matrix contents for debugging.

#### `TestConstants<Scalar>`

A trait struct that provides per-scalar-type default tolerances for `test_jacobian()`. This allows the same test code to work with both `double` and `float` precision without manually tuning parameters:

```cpp
template <class Scalar> struct TestConstants;

template <> struct TestConstants<double> {
  static constexpr double epsilon  = 1e-8;   // Finite-difference step
  static constexpr double max_norm = 1e-3;   // Comparison tolerance
};

template <> struct TestConstants<float> {
  static constexpr double epsilon  = 1e-2;   // Larger step for float precision
  static constexpr double max_norm = 1e-2;   // Looser tolerance for float
};
```

The `double` specialisation uses a tight step (`1e-8`) close to $\sqrt{\text{machine epsilon}}$, which minimises the combined truncation and rounding error of central differences. The `float` specialisation uses a much larger step (`1e-2`) to account for the reduced 23-bit mantissa.

### Build System

This section describes how gtest is integrated into the Basalt build system via CMake, how test executables are defined and discovered, the conditional compilation mechanism, and how to build and run tests.

#### CMake Integration Architecture

The gtest build integration is structured in three layers:

**Layer 1 — Root `CMakeLists.txt`** (lines 481–483):
```cmake
enable_testing()
add_subdirectory(thirdparty/basalt-headers/test)  # Brings in gtest + test utilities
add_subdirectory(test)                             # Brings in basalt unit tests
```
`enable_testing()` activates CTest support, making the `ctest` command available from the build directory. The two `add_subdirectory()` calls bring in the vendored gtest and the Basalt-specific tests, respectively.

**Layer 2 — `thirdparty/basalt-headers/test/CMakeLists.txt`**:
```cmake
if(NOT TARGET gtest_main)
    add_subdirectory(googletest EXCLUDE_FROM_ALL)
endif(NOT TARGET gtest_main)

add_library(basalt-headers-test-utils INTERFACE)
target_link_libraries(basalt-headers-test-utils INTERFACE gtest)
target_include_directories(basalt-headers-test-utils INTERFACE include)
target_compile_definitions(basalt-headers-test-utils INTERFACE BASALT_ENABLE_BOUNDS_CHECKS)
```
This layer conditionally compiles the vendored googletest source tree (only if `gtest_main` hasn't already been defined by another project). `EXCLUDE_FROM_ALL` prevents gtest from being built unless a test target needs it. It also creates the `basalt-headers-test-utils` INTERFACE library, which provides the `test_utils.h` header (containing `test_jacobian()` and `TestConstants`) and enables bounds checking via `BASALT_ENABLE_BOUNDS_CHECKS`.

**Layer 3 — `test/CMakeLists.txt`**:
```cmake
cmake_minimum_required(VERSION 3.10...3.18)

include_directories(../thirdparty/basalt-headers/test/include)

add_executable(test_nfr src/test_nfr.cpp)
target_link_libraries(test_nfr gtest gtest_main basalt)

# ... (other test executables follow the same pattern) ...

enable_testing()
include(GoogleTest)
gtest_add_tests(TARGET test_nfr AUTO)
```

Each test executable links three libraries:
| Library | Purpose |
|---|---|
| `gtest` | The core gtest framework (assertions, test registration, runner) |
| `gtest_main` | Provides a default `main()` that calls `RUN_ALL_TESTS()` |
| `basalt` | The Basalt library under test (provides all the classes and functions being validated) |

The `include_directories()` call makes `test_utils.h` available to all test sources.

#### Test Discovery

gtest provides two CMake mechanisms for registering tests with CTest:

| Mechanism | When tests are found | Used by |
|---|---|---|
| `gtest_add_tests(TARGET <name> AUTO)` | **Configure time** — CMake parses the source file for `TEST()` macros | `test/CMakeLists.txt` (Basalt tests) |
| `gtest_discover_tests(<name>)` | **Build time** — runs the compiled binary with `--gtest_list_tests` | `thirdparty/basalt-headers/test/CMakeLists.txt` (basalt-headers tests) |

The Basalt test directory uses the legacy `gtest_add_tests()` approach. The modern `gtest_discover_tests()` is available but currently commented out:
```cmake
# From test/CMakeLists.txt
#gtest_discover_tests(test_nfr DISCOVERY_TIMEOUT 60)   # Commented out
gtest_add_tests(TARGET test_nfr AUTO)                   # Active
```

The practical difference: `gtest_add_tests()` uses regex to find `TEST()` macros in the source, so it works without building the binary first. `gtest_discover_tests()` is more accurate (handles parameterised and generated tests) but requires the binary to be compiled before CTest can list its tests.

#### Conditional Compilation (`BASALT_INSTANTIATIONS_*`)

Basalt's template-heavy architecture means many classes (e.g., `LandmarkDatabase<Scalar>`, `BundleAdjustmentBase<Scalar>`) are only explicitly instantiated for scalar types enabled at CMake configure time. Two CMake options control this:

```cmake
# From root CMakeLists.txt (lines 243–251)
option(BASALT_INSTANTIATIONS_DOUBLE "Instatiate templates for Scalar=double." ON)
option(BASALT_INSTANTIATIONS_FLOAT  "Instatiate templates for Scalar=float."  ON)

if(BASALT_INSTANTIATIONS_DOUBLE)
  list(APPEND BASALT_COMPILE_DEFINITIONS BASALT_INSTANTIATIONS_DOUBLE)
endif()
```

These definitions are propagated to all targets via `target_compile_definitions(basalt PUBLIC ${BASALT_COMPILE_DEFINITIONS})`. Tests that depend on a specific instantiation must guard their `TEST()` macros:

```cpp
// From test/src/test_linearization.cpp
#ifdef BASALT_INSTANTIATIONS_DOUBLE
TEST(LinearizationTestSuite, VoNoMargLinearizationTest) {
  // Uses BundleAdjustmentBase<double>, which is only available
  // when BASALT_INSTANTIATIONS_DOUBLE is defined
  basalt::BundleAdjustmentBase<double> estimator;
  // ...
}
#endif
```

The following test files use this guard:
| File | Guard |
|---|---|
| `test/src/test_linearization.cpp` | `BASALT_INSTANTIATIONS_DOUBLE` (all 4 tests) |
| `test/src/test_vio.cpp` | `BASALT_INSTANTIATIONS_DOUBLE` (2 of 4 tests) |
| `test/src/test_qr.cpp` | `BASALT_INSTANTIATIONS_DOUBLE` (1 of 3 tests) |

Tests in `test_nfr.cpp`, `test_patch.cpp`, and `test_spline_opt.cpp` do **not** use this guard because they operate on concrete types (e.g., `Sophus::SE3d`) rather than the library's templated classes.

#### Building Tests

From a configured build directory:

```bash
# Configure (if not already done) — both instantiation options default to ON
cmake -S /path/to/basalt -B build \
      -DBASALT_INSTANTIATIONS_DOUBLE=ON \
      -DBASALT_INSTANTIATIONS_FLOAT=ON

# Build a single test executable
cd build
make test_nfr

# Build all test executables (alongside the main library)
make
```

Each `make <test_target>` compiles the test source, links it against `gtest`, `gtest_main`, and `basalt`, and produces a binary under `build/test/`.

#### Running Tests

Tests can be run in three ways:

**1. Direct execution** — runs the binary and prints gtest output:
```bash
./test/test_nfr
```

**2. Via CTest** — the CMake test runner:
```bash
ctest                       # Run all registered tests
ctest -R test_nfr           # Run tests matching regex "test_nfr"
ctest -V                    # Verbose: show full gtest output for each test
ctest -j$(nproc)            # Run tests in parallel
```

**3. gtest command-line flags** — passed directly to the binary:
```bash
# Run only the RelPoseTest within PreIntegrationTestSuite
./test/test_nfr --gtest_filter=PreIntegrationTestSuite.RelPoseTest

# List all tests without running them
./test/test_nfr --gtest_list_tests

# Run each test 5 times (useful for catching flaky tests)
./test/test_nfr --gtest_repeat=5

# Shuffle test execution order
./test/test_nfr --gtest_shuffle
```

---

## 3. Tests Implemented

<!-- TODO Review and Update -->

This section documents each existing test executable in detail. For each test, we describe: the mathematical formulation being tested, the test architecture, data generation strategy, success criteria, and the correlation between code and mathematics.

### 3.1 `test_linearization` — Bundle Adjustment Linearisation Consistency

**File:** `test/src/test_linearization.cpp` (14,407 bytes, 4 tests)

#### 3.1.1 Mathematical Background

Bundle adjustment minimises the total reprojection error over keyframe poses and landmark positions. The Gauss-Newton normal equations yield:

$$\mathbf{H} \boldsymbol{\xi} = \mathbf{b}$$

where $\mathbf{H} = \mathbf{J}^T \mathbf{W} \mathbf{J}$ and $\mathbf{b} = -\mathbf{J}^T \mathbf{W} \mathbf{r}$. Basalt implements three linearisation strategies that should produce mathematically identical results:

- **ABS_QR**: Absolute pose parameterisation with QR-based landmark elimination.
- **ABS_SC**: Absolute pose parameterisation with Schur complement landmark elimination.
- **REL_SC**: Relative pose parameterisation with Schur complement.

All three strategies eliminate landmark variables via the Schur complement to produce a pose-only system, but differ in how they factor and accumulate the contributions.

#### 3.1.2 Test Architecture

**Helper functions:**
- `get_vo_estimator<Scalar>(num_frames, estimator, aom)` (line 11): Creates a synthetic VO problem with `num_frames` keyframes, each hosting 10 random 3D landmarks. Landmarks are projected into both stereo cameras with small Gaussian noise. Returns a populated `BundleAdjustmentBase<Scalar>` and `AbsOrderMap`.
- `get_vo_estimator_with_marg<Scalar>(num_frames, estimator, aom, mld)` (line 84): Extends the above with a synthetic marginalisation prior (identity H scaled by 1e6, random b) over the first two frames.

**Test Cases:**

| Test | Lines | Description |
|---|---|---|
| `VoNoMargLinearizationTest` | 111–175 | Compares H, b, error across ABS_QR, ABS_SC, REL_SC without marginalisation |
| `VoMargLinearizationTest` | 177–243 | Same comparison with marginalisation prior |
| `VoMargBacksubstituteTest` | 244–328 | Verifies landmark backsubstitution produces correct H and b |
| `VoMargSqrtLinearizationTest` | 329–end | Tests the square-root linearisation variant |

**Success Criteria:** `EXPECT_LE((H_a - H_b).norm(), tolerance)` where tolerance is `1e-8` for H matrices and the relative error norm is bounded.

**Data Pipeline:**
```
Random 3D points → Project into cameras → Add noise → Build lmdb
                                                     → Linearise 3 ways
                                                     → Compare H, b, error
```

### 3.2 `test_vio` — VIO IMU Preintegration and Nullspace

**File:** `test/src/test_vio.cpp` (14,251 bytes, 4 tests)

#### 3.2.1 Mathematical Background

The VIO factor graph has a 7-DOF nullspace: 3 translations + 3 rotations (global pose) + 1 scale. This means the information matrix $\mathbf{H}$ should have exactly 7 zero eigenvalues. Additionally, the Jacobians of visual and IMU residuals must be correct for the optimiser to converge.

#### 3.2.2 Test Cases

| Test | Lines | Description |
|---|---|---|
| `ImuNullspace2Test` | 28–160 | Creates 2-frame VIO problem with simulated IMU on a spline trajectory; verifies $\mathbf{H}$ nullspace has 7 dimensions |
| `ImuNullspace3Test` | 162–313 | Same with 3 frames; more complex coupling |
| `RelPoseTest` | 315–359 | Verifies `computeRelPose()` Jacobian via finite differences |
| `LinearizePointsTest` | 361–end | Verifies `linearizePoint()` Jacobian for landmark reprojection |

**Data Generation:** Simulated trajectory via `basalt::Se3Spline<5>` (quintic B-spline); IMU data sampled at 200 Hz with Gaussian noise added from accelerometer and gyroscope noise models.

**Success Criteria:**
- Nullspace: eigenvalues of $\mathbf{H}$ sorted ascending; first 7 must be ≤ tolerance.
- Jacobians: `test_jacobian()` against finite differences.

### 3.3 `test_nfr` — NFR Factor Jacobians

**File:** `test/src/test_nfr.cpp` (3,697 bytes, 4 tests)

#### 3.3.1 Mathematical Background

The Non-Linear Factor Recovery module produces four types of residuals:
- `relPoseError`: $\mathbf{r} = \text{Log}(\mathbf{T}_{i,j} \cdot \mathbf{T}_{w,j}^{-1} \mathbf{T}_{w,i}) \in \mathbb{R}^6$
- `absPositionError`: $\mathbf{r} = \mathbf{T}_{w,i}.\text{translation} - \mathbf{p}_{\text{meas}} \in \mathbb{R}^3$
- `yawError`: $r = \text{yaw error scalar} \in \mathbb{R}^1$
- `rollPitchError`: $\mathbf{r} = (\Delta\mathbf{R} \cdot (-\hat{\mathbf{z}}))_{x,y} \in \mathbb{R}^2$

Each has an analytical Jacobian with respect to the SE3 pose perturbation. The tests verify these analytically-computed Jacobians against numerical finite differences.

#### 3.3.2 Test Architecture

Each test follows the same pattern:
1. Generate random SE3 poses.
2. Compute the residual and its analytical Jacobian.
3. Call `test_jacobian()` with a lambda that computes the residual at a perturbed pose.
4. The helper internally applies central finite differences and compares.

**Success Criteria:** `test_jacobian()` internally uses `isApprox()` with tolerance `1e-3`.

### 3.4 `test_qr` — QR Decomposition and Schur Complement

**File:** `test/src/test_qr.cpp` (3,773 bytes, 3 tests)

#### 3.4.1 Mathematical Background

The Schur complement eliminates a subset of variables from the normal equations. Three equivalent methods:
1. **SqToSq**: Dense $\mathbf{H}^* = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha}$
2. **SqToSqrt**: Same input, but output is the square-root (Jacobian) form.
3. **SqrtToSqrt**: Input is the Jacobian $\mathbf{J}$; QR decomposition eliminates variables.

All three should produce the same solution when the reduced system is solved.

#### 3.4.2 Test Architecture

A random 10×6 Jacobian with a deliberately duplicated column (rank deficiency) is created. All three `MargHelper` methods are applied. The solutions are compared via `isApprox()`.

**Success Criteria:** `EXPECT_TRUE(sol_qr.isApprox(sol_sc))`.

### 3.5 `test_patch` — Optical Flow Patch

**File:** `test/src/test_patch.cpp` (2,428 bytes, 2 tests)

Tests image gradient interpolation and the SE2 Jacobian of the patch tracking residual using a synthetic smooth function $f(x,y) = \sin(x/100 + y/20)$.

### 3.6 `test_spline_opt` — Spline Optimisation

**File:** `test/src/test_spline_opt.cpp` (3,557 bytes, 1 test)

Creates a random quintic SE3 spline, generates synthetic IMU and pose measurements, runs 10 optimisation iterations, and verifies that recovered calibration parameters (accelerometer bias, gyroscope bias, gravity vector) match ground truth.

---

## 4. Local Mapper Integration and Unit Tests

This section plans the implementation of all new tests specified in §11.5 of `doc/LocalMapper.md`. Each test is designed to validate a specific component of the LocalMapper infrastructure added in §11.1–11.4.

### 4.1 `UnionFindAddIndexTest`

#### 4.1.1 Mathematical Formulation and Background

The Union-Find (Disjoint Set Union) data structure partitions a set of elements into non-overlapping subsets. `AddIndex()` extends the forest by appending a single new node as its own root. The invariant is: after `AddIndex()`, the new node's parent is itself, its rank is 0, and its size is 1.

**Motivation:** The incremental local mapper adds feature nodes one-at-a-time as new matches arrive (unlike the batch `InitSets(n)` used by the offline mapper). `AddIndex()` must return the correct dense index and leave the existing forest undisturbed.

#### 4.1.2 Data Pipeline

```
Input:  Empty UnionFind
        → AddIndex() × 5
        → Union(0,1), Union(2,3)
Output: GetNumNodes() == 5
        Find(0) == Find(1)
        Find(2) == Find(3)
        Find(4) is a singleton root
```

#### 4.1.3 Complexity Analysis

`AddIndex()`: O(1) amortised (vector push_back). `Find()`: O(α(n)) with path compression.

#### 4.1.4 Code Implementation

```cpp
TEST(LocalMapperTestSuite, UnionFindAddIndexTest) {
  UnionFind uf;
  // Add 5 nodes incrementally
  ASSERT_EQ(uf.AddIndex(), 0);
  ASSERT_EQ(uf.AddIndex(), 1);
  ASSERT_EQ(uf.AddIndex(), 2);
  ASSERT_EQ(uf.AddIndex(), 3);
  ASSERT_EQ(uf.AddIndex(), 4);
  ASSERT_EQ(uf.GetNumNodes(), 5);

  // Union some
  uf.Union(0, 1);
  uf.Union(2, 3);

  // Verify sets
  EXPECT_EQ(uf.Find(0), uf.Find(1));
  EXPECT_EQ(uf.Find(2), uf.Find(3));
  EXPECT_NE(uf.Find(0), uf.Find(2));
  EXPECT_NE(uf.Find(0), uf.Find(4));
  EXPECT_NE(uf.Find(2), uf.Find(4));
  // Node 4 is its own root
  EXPECT_EQ(uf.Find(4), 4u);
}
```

### 4.2 `UnionFindInvalidateRootTest`

#### 4.2.1 Mathematical Formulation and Background

`InvalidateRoot(root_idx)` sets the parent of a root to `InvalidIndex()`. Subsequent `Find()` calls on any node in that set must return `InvalidIndex()` via path compression. This is the mechanism that "deletes" a track from the DSU without physically removing nodes.

**Motivation:** When a track is found to have conflicting observations (same image twice) or falls below minimum length, its root is invalidated. All descendants must silently resolve to invalid on the next `Find()`.

#### 4.2.2 Data Pipeline

```
Input:  UnionFind with nodes 0–4, Union(0,1), Union(1,2)
        → InvalidateRoot(Find(0))
Output: Find(0) == Find(1) == Find(2) == InvalidIndex()
        Find(3) and Find(4) remain valid singletons
```

#### 4.2.3 Code Implementation

```cpp
TEST(LocalMapperTestSuite, UnionFindInvalidateRootTest) {
  UnionFind uf;
  for (int i = 0; i < 5; ++i) uf.AddIndex();
  uf.Union(0, 1);
  uf.Union(1, 2);

  auto root = uf.Find(0);
  ASSERT_NE(root, UnionFind::InvalidIndex());
  uf.InvalidateRoot(root);

  // All nodes in the invalidated set resolve to InvalidIndex
  EXPECT_EQ(uf.Find(0), UnionFind::InvalidIndex());
  EXPECT_EQ(uf.Find(1), UnionFind::InvalidIndex());
  EXPECT_EQ(uf.Find(2), UnionFind::InvalidIndex());

  // Other nodes unaffected
  EXPECT_NE(uf.Find(3), UnionFind::InvalidIndex());
  EXPECT_NE(uf.Find(4), UnionFind::InvalidIndex());
}
```

### 4.3 `TrackBuilderAddNewMatchesBrandNew`

#### 4.3.1 Mathematical Formulation and Background

`AddNewMatches()` incrementally extends the track set. When presented with an entirely new pair of image-feature observations that don't overlap with any existing tracks, a brand-new track should be created with a fresh `TrackId` from `mpNextTrackId++`.

**Motivation:** Every iteration of the local mapper brings in new keyframes with new feature matches. The tracker must assign stable, unique IDs to each new track.

#### 4.3.2 Data Pipeline

```
Input:  Empty TrackBuilder
        → Build(initial_matches)        // Creates tracks 0, 1, 2
        → AddNewMatches(new_matches)     // new_matches touch unseen features
Output: updated_tracks contains new TrackId ≥ 3
        retired_ids is empty (no merges)
```

#### 4.3.3 Complexity Analysis

`AddNewMatches`: O(|new_feats| · α(n) + |map_node_to_index|) for the union-find operations and export.

#### 4.3.4 Code Implementation

Constructs synthetic `Matches` (TBB concurrent map) with fabricated `TimeCamId` pairs, calls `Build` then `AddNewMatches`, and verifies:
- `updated_tracks` has exactly the expected number of new tracks.
- `retired_ids` is empty.
- Each new track's `TrackId` is ≥ the previous `mpNextTrackId`.

### 4.4 `TrackBuilderAddNewMatchesAppend`

Tests that adding a match between an existing tracked feature and a new feature extends the existing track (same `TrackId`) rather than creating a new one.

### 4.5 `TrackBuilderAddNewMatchesMergeOneLandmark`

Tests the survivor-preference heuristic: when merging two tracks where only one has a landmark in `lmdb`, the landmark-bearing track's `TrackId` survives and the other's `TrackId` appears in `retired_ids`.

Uses a mock `LandmarkDatabase<double>` with a single landmark registered under one of the two tracks.

### 4.6 `TrackBuilderAddNewMatchesMergeBothLandmarks`

Tests the merge case where both tracks have landmarks. The larger track (by `m_cc_size`) wins; the smaller's `TrackId` appears in `retired_ids`.

### 4.7 `TrackBuilderCompactTreeTest`

After building tracks, invalidating some, and calling `CompactTree()`:
- The DSU should be densely indexed `[0, N)`.
- `mpUFNodeIndexToTrackIdMap` should map new roots to the same `TrackId` values as before.
- `map_node_to_index` should contain only live feature pairs.

### 4.8 `HashBowStlRemoveKeyframesTest`

#### 4.8.1 Mathematical Formulation and Background

The `HashBowStl<N>` inverted index maps BoW hash words to `(TimeCamId, weight)` entries. `RemoveKeyframes()` must erase all references to the given keyframes from both the inverted index and the reverse index (`mpTcidHashIndex`).

**Motivation:** When the local mapper culls a keyframe, its BoW entries must be removed so that subsequent queries don't return stale matches.

#### 4.8.2 Data Pipeline

```
Input:  HashBowStl<256> with 3 KFs added
        → RemoveKeyframes([kf_1])
Output: querry_database from kf_2 does not return kf_1
        mpTcidHashIndex does not contain kf_1
```

#### 4.8.3 Code Implementation

Creates synthetic 256-bit descriptors, computes BoW vectors, adds 3 keyframes, removes 1, then queries and verifies the removed KF does not appear in results.

### 4.9 Integration Test: `basalt_slam`

#### 4.9.1 Background

The integration test exercises the full SLAM pipeline: optical flow → VIO → LocalMapper, using a recorded EuRoC dataset. It validates that:
1. The system initialises and runs without crashes.
2. The local mapper receives MargData and populates `frame_poses`.
3. The local map size stays bounded by `mpMaxLocalMapSize`.
4. Shutdown is clean (no hanging threads, no deadlocks).

#### 4.9.2 Classes Involved

The integration test exercises the following classes and their interactions. Understanding their roles is essential to reading the data pipeline and pseudo code that follow.

| Class | Header | Role in the Integration Test |
|---|---|---|
| `Controller` | `include/basalt/controller.h` | **Top-level orchestrator.** Creates, wires, and owns the full pipeline (OpticalFlow → VIO → LocalMapper). Exposes `GrabImage()` / `GrabIMU()` for data ingestion and `Stop()` for cascade shutdown. Replaces the manual queue wiring in `src/vio.cpp`. |
| `OpticalFlowBase` | `include/basalt/optical_flow/optical_flow.h` | **Frontend feature tracker.** Receives raw images via `input_queue`, runs frame-to-frame patch tracking, and pushes `OpticalFlowResult` to `output_queue`. Created internally by `Controller::initialize()` via `OpticalFlowFactory`. Runs on thread **T1** if `useProducerConsumerArchitecture=true`, otherwise vio and optical flow runs on the same thread. |
| `VioEstimatorBase<double>` | `include/basalt/vi_estimator/vio_estimator.h` | **VIO backend.** Receives optical flow results and IMU data, runs a fixed-lag sliding-window optimiser, and outputs pose estimates (`out_state_queue`) and marginalisation data (`out_marg_queue`). Created internally by `Controller::initialize()` via `VioEstimatorFactory`. The concrete type is `SqrtKeypointVioEstimator<double>` for VIO mode. Runs on thread **T2** in `useProducerConsumerArchitecture=true`, otherwise vio and optical flow tracking runs on the same thread. |
| `LocalMapper` | `include/basalt/vi_estimator/local_mapper.h` | **Local mapping backend.** Receives `MargData` from VIO, runs BoW matching, track building, bundle adjustment, and keyframe culling. Feeds refined poses back to VIO via `QueuePoseUpdates()`. Inherits from `class NfrMapper` which implements the following inheritance chain: `NfrMapper` → `ScBundleAdjustmentBase<double>` → `BundleAdjustmentBase<double>`. `BundleAdjustmentBase<double>` provides the `frame_poses` map used in test assertions. Created by `Controller::initialize()` when `useProducerConsumerArchitecture=true`. Runs on thread **T3**. |
| `DatasetIoFactory` | `include/basalt/io/dataset_io.h` | **Dataset loader factory.** `getDatasetIo("euroc")` returns a `DatasetIoInterface` that reads the EuRoC MAV dataset format (timestamped stereo images + IMU CSV). |
| `VioDataset` | `include/basalt/io/dataset_io.h` | **Dataset accessor.** Provides `get_image_timestamps()`, `get_image_data(t_ns)`, `get_gyro_data()`, and `get_accel_data()` for sequential iteration over the recorded dataset. |
| `Calibration<double>` | `include/basalt/calibration/calibration.hpp` | **Camera calibration.** Loaded from a JSON file (e.g., `data/euroc_ds_calib.json`). Contains intrinsics, extrinsics (`T_i_c`), and distortion models. Passed to `Controller` at construction. |
| `VioConfig` | `include/basalt/utils/vio_config.h` | **Pipeline configuration.** Loaded from a JSON file (e.g., `data/euroc_config.json`). Controls optical flow parameters, VIO window size, optimisation iterations, etc. Loaded by `Controller::load_config()`. |
| `OpticalFlowInput` | `include/basalt/optical_flow/optical_flow.h` | **Image data wrapper.** Holds a timestamp (`t_ns`) and per-camera image data (`img_data`). Constructed in the image feed thread and pushed via `Controller::GrabImage()`. |
| `ImuData<double>` | `include/basalt/utils/imu_types.h` | **IMU measurement wrapper.** Holds a timestamp, accelerometer, and gyroscope readings. Constructed in the IMU feed thread and pushed via `Controller::GrabIMU()`. |
| `PoseVelBiasState<double>` | `include/basalt/utils/imu_types.h` | **VIO output state.** Contains the estimated pose (`T_w_i`), velocity, and IMU biases at a given timestamp. Buffered by `Controller`'s internal pose processing thread (**T4**). |
| `MargData` | `include/basalt/vi_estimator/vio_estimator.h` | **Marginalisation data packet.** Contains the marginalised keyframe's pose, Hessian, observations, and associated data. Pushed by VIO when a keyframe is marginalised; consumed by `LocalMapper::IngestMargData()`. |

**Thread model:** When `useProducerConsumerArchitecture=true`, `Controller::initialize()` creates four concurrent threads:

| Thread | Owner | Role |
|---|---|---|
| **T1** | `OpticalFlowBase::processing_thread` | Frontend: raw images → optical flow results |
| **T2** | `VioEstimatorBase::processing_thread` | Backend: optical flow + IMU → poses + MargData |
| **T3** | `LocalMapper::mpLocalMappingThread` | Mapping: MargData → refined poses (feedback to T2) |
| **T4** | `Controller::pose_processing_thread_` | Pose consumer: buffers latest pose for `GetLatestPose()` |

All inter-thread communication uses `tbb::concurrent_bounded_queue` (lock-free bounded FIFO). Shutdown is deterministic: `Controller::Stop()` pushes `nullptr` into the optical flow input queue; each stage propagates `nullptr` downstream, causing the next stage to exit its processing loop.

#### 4.9.3 Data Pipeline

```
EuRoC dataset (images + IMU)
    → feed_images thread: Controller::GrabImage(data)
    → feed_imu thread:    Controller::GrabIMU(data)
        → [T1] OpticalFlow::input_queue → processingLoop() → output_queue
        → [T2] VIO::vision_data_queue + imu_data_queue → ProcessFrame()
            → out_state_queue → [T4] Controller::process_pose_queue_loop()
            → out_marg_queue  → [T3] LocalMapper::MapLocally()
                                     → IngestMargData → MatchLocal → build_tracks
                                     → setup_opt → optimize → CullRedundantKeyframes
                                     → mpVioPoseUpdateCallback (feedback to T2)
    → Controller::Stop()  [cascade nullptr shutdown]
    → Assert: local_mapper_->frame_poses.size() > 0
    → Assert: local_mapper_->frame_poses.size() ≤ mpMaxLocalMapSize
```

The executable `src/basalt_slam.cpp` adapts the `src/vio.cpp` reference implementation: it retains the argument parsing, dataset loading, and data feeding logic, but replaces the manual pipeline wiring and Pangolin GUI with a `Controller` object that owns the entire pipeline.

#### 4.9.4 Code Implementation

The implementation adapts `src/vio.cpp`. The following pseudo code describes the structure of `src/basalt_slam.cpp`, annotating what is **retained** from `vio.cpp`, what is **removed**, and what is **new**.

**Retained from `vio.cpp`:**
- CLI argument parsing (`CLI::App`): `--cam-calib`, `--dataset-path`, `--dataset-type`, `--config-path`, `--num-threads`, `--max-frames`.
- TBB global thread control (`tbb::global_control`).
- Dataset loading via `DatasetIoFactory::getDatasetIo(dataset_type)`.
- Image and IMU feed threads (`feed_images`, `feed_imu`) — adapted to use `Controller` instead of directly pushing to `opt_flow_ptr_` and `vio`.

**Removed from `vio.cpp`:**
- All Pangolin includes and GUI code (`#include <pangolin/...>`, `draw_image_overlay`, `draw_scene`, `draw_plots`, `pangolin::Var<>` variables, the GUI event loop).
- Visualisation globals (`vis_map`, `out_vis_queue`, `vio_t_ns`, `vio_t_w_i`, etc.).
- The visualisation consumer thread (`t3`) and state accumulation thread (`t4`) — `Controller` handles pose buffering internally via its `process_pose_queue_loop()` thread.
- `MargDataSaver` — the integration test does not save marginalisation data to disk.
- Trajectory saving and alignment logic (`saveTrajectoryButton`, `alignButton`).

**New in `basalt_slam.cpp`:**
- `Controller` object replaces the separate `opt_flow_ptr_` and `vio` objects.
- `Controller::initialize()` with `useProducerConsumerArchitecture=true` creates the full four-thread pipeline (OpticalFlow, VIO, LocalMapper, PoseConsumer), otherwise, only two threads are created (OpticalFlow and VIO in one and LocalMapper in the other).
- Post-pipeline assertions on `local_mapper_->frame_poses` for test validation.

```
PSEUDO CODE: src/basalt_slam.cpp
─────────────────────────────────

INCLUDES:
    <basalt/controller.h>              // NEW — replaces direct pipeline wiring
    <basalt/io/dataset_io.h>           // RETAINED from vio.cpp
    <basalt/utils/vio_config.h>        // RETAINED
    <CLI/CLI.hpp>                      // RETAINED
    <tbb/global_control.h>             // RETAINED
    // NOTE: No <pangolin/...> includes — GUI is removed entirely

GLOBALS:
    // REMOVED: All pangolin variables (show_frame, show_flow, camera, plotter, etc.)
    // REMOVED: vis_map, out_vis_queue, vio_data_log, error_data_log
    // REMOVED: vio_t_ns, vio_t_w_i, vio_T_w_i, gt_t_ns, gt_t_w_i
    // REMOVED: marg_data_path, timestamp_to_id
    // REMOVED: opt_flow_ptr, vio (replaced by Controller)
    // REMOVED: step_by_step, continue_btn (no GUI interaction)
    // RETAINED: max_frames, terminate (for feed thread control)
    // NEW: controller — shared_ptr<basalt::Controller>

────────────────────────────────────────────────────────────────────────

FUNCTION main(argc, argv):

    // ─── 1. ARGUMENT PARSING (RETAINED from vio.cpp) ─────

    PARSE CLI arguments:
        --cam-calib     → cam_calib_path    (required)
        --dataset-path  → dataset_path      (required)
        --dataset-type  → dataset_type      (required, e.g. "euroc")
        --config-path   → config_path       (optional)
        --num-threads   → num_threads       (optional, default 0 = unlimited)
        --max-frames    → max_frames        (optional, default 0 = unlimited)
        // REMOVED: --show-gui, --marg-data, --step-by-step, --print-queue,
        //          --save-trajectory, --save-groundtruth, --use-imu, --use-double

    // ─── 2. TBB THREAD CONTROL (RETAINED from vio.cpp) ───────────────

    IF num_threads > 0:
        CREATE tbb::global_control(max_allowed_parallelism, num_threads)

    // ─── 3. CONTROLLER CREATION (NEW — replaces vio.cpp's manual wiring) ───

    // In vio.cpp this was done in three separate steps:
    //   1. load_data(cam_calib_path)     → loads calib
    //   2. OpticalFlowFactory::getOpticalFlow(config, calib) → creates frontend
    //   3. VioEstimatorFactory::getVioEstimator(config, calib, ...) → creates VIO
    //   4. Manual queue wiring: opt_flow_ptr->output_queue = &vio->vision_data_queue
    //   5. vio->out_state_queue = &out_state_queue
    //
    // Controller encapsulates all of this:

    controller = NEW Controller(config_path, cam_calib_path, SlamMode::VIO)
    controller->load_config()
    calib = controller->GetCalibration();
    Eigen::Matrix<double, 3, 1> bg;
    Eigen::Matrix<double, 3, 3> sg;
    Eigen::Matrix<double, 3, 1> ba;
    Eigen::Matrix<double, 3, 3> sa;
    calib.calib_gyro_bias.getBiasAndScale(bg, sg);
    calib.calib_accel_bias.getBiasAndScale(ba, sa);
    // convert ba and bg to Eigen::Vector3d for passing to initialize()
    controller->initialize(
        t_ns    = 0,                               // start at origin
        T_w_i   = SE3d::Identity(),                // identity pose
        vel_w_i = Vector3d::Zero(),                // zero velocity
        bg,                                        // to be read and converted using calib.calib_gyro_bias.getBiasAndScale(...)
        ba,                                        // to be read and converted from calib.calib_accel_bias.getBiasAndScale(...)
        useProducerConsumerArchitecture = false    // ← KEY: enables 2-thread pipeline, setting to true enables 4 thread pipeline
    )
    // This internally:
    //   - Creates OpticalFlowBase (frontend, thread T1)
    //   - Creates VioEstimatorBase<double> (VIO backend, thread T2)
    //   - Wires: opt_flow → VIO vision queue
    //   - Wires: VIO out_state_queue → Controller's pose buffer (thread T4)
    //   - Wires: VIO out_marg_queue → local_map_input_queue
    //   - Creates LocalMapper, sets marg queue and pose-feedback callback (thread T3)
    //   - Starts all threads needed

    // ─── 4. DATASET LOADING (RETAINED from vio.cpp) ──────────────────

    dataset_io = DatasetIoFactory::getDatasetIo(dataset_type)
    dataset_io->read(dataset_path)
    vio_dataset = dataset_io->get_data()

    // ─── 5. DATA FEEDING ──────────────────────

    // In vio.cpp:
    //   feed_images() pushes to opt_flow_ptr->input_queue
    //   feed_imu()    pushes to vio->imu_data_queue
    //
    // In basalt_slam.cpp:
    //   feed_data() iterates over the frame datasets and filters out imu data till the current frame and processes both the imu and frame data while ensuring time synchronisation.

    vio_estimator = controller->GetVIO();
    THREAD t1 = feed_data:
        k = 0;
        FOR i IN 0..vio_dataset->get_image_timestamps().size():
            IF vio_estimator->finished OR terminate OR (max_frames > 0 AND i >= max_frames):
                BREAK

            data = NEW OpticalFlowInput
            data->t_ns     = vio_dataset->get_image_timestamps()[i]
            data->img_data = vio_dataset->get_image_data(data->t_ns)

            while(k < get_gyro_data().size() && vio_dataset->get_gyro_data()[k].timestamp_ns <= data->t_ns){
                imu_data = NEW ImuData<double>
                imu_data->t_ns  = vio_dataset->get_gyro_data()[k].timestamp_ns
                imu_data->accel  = vio_dataset->get_accel_data()[k].data
                imu_data->gyro   = vio_dataset->get_gyro_data()[k].data
                k++;

                controller->GrabIMU(imu_data)      // NEW: was vio->imu_data_queue.push(imu_data)
            }
            if(k<vio_dataset->get_gyro_data().size()){
                imu_data = NEW ImuData<double>
                imu_data->t_ns  = vio_dataset->get_gyro_data()[k].timestamp_ns
                imu_data->accel  = vio_dataset->get_accel_data()[k].data
                imu_data->gyro   = vio_dataset->get_gyro_data()[k].data
                k++;
                controller->GrabIMU(imu_data)      // NEW: was vio->imu_data_queue.push(imu_data)
            }
            Sophus::SE3f tcw;
            controller->TrackMonocular(data, tcw)    // NEW: was opt_flow_ptr->input_queue.push(data)

        
        // NOTE: Do NOT push nullptr here — Controller::Stop() handles the
        // cascade sentinel. In vio.cpp, feed_images pushed nullptr to
        // opt_flow_ptr->input_queue to signal end-of-stream. Here, Stop()
        // does this.

    // REMOVED from vio.cpp:
    //   thread t3 (visualisation consumer) — Controller has no vis queue
    //   thread t4 (state accumulation)     — Controller buffers poses internally
    //   thread t5 (queue printer)          — not needed for test

    // ─── 6. WAIT FOR DATA FEEDING TO COMPLETE ────────────────────────

    t1.join()    // Data feed finished

    // At this point all data has been pushed into the pipeline, but the
    // pipeline threads (T1–T4) are still processing the tail end of the
    // data. Controller::Stop() will wait for them to drain.

    // ─── 7. SHUTDOWN (NEW — replaces vio.cpp's manual join sequence) ──

    // In vio.cpp:
    //   vio->maybe_join()
    //   vio->drain_input_queues()
    //   t1.join(); t2.join(); t3->join(); t4.join(); t5->join()
    //
    // Controller::Stop() handles the full cascade:
    //   1. Push nullptr to optical flow input queue
    //   2. OpticalFlow propagates nullptr → VIO
    //   3. VIO propagates nullptr → out_marg_queue + out_state_queue
    //   4. LocalMapper receives nullptr → exits MapLocally loop
    //   5. local_mapper_->Stop() joins T3
    //   6. vio_estimator_->maybe_join() joins T2
    //   7. opt_flow_ptr_.reset() joins T1 (via destructor)
    //   8. pose_processing_thread_ joins T4

    controller->Stop()

    // ─── 8. TEST ASSERTIONS (NEW — not in vio.cpp) ───────────────────

    // Access the LocalMapper through Controller's local_mapper_ member.
    // LocalMapper inherits frame_poses from BundleAdjustmentBase<double>
    // (via NfrMapper → ScBundleAdjustmentBase → BundleAdjustmentBase).
    //
    // frame_poses is:
    //   Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses;
    //
    // It contains one entry per keyframe currently in the local map.

    local_mapper = controller->GetLocalMapper();

    // Assertion 1: The local mapper processed at least some data
    ASSERT local_mapper->frame_poses.size() > 0
        "LocalMapper should have at least one keyframe after processing"

    // Assertion 2: The local map respects the size bound
    ASSERT local_mapper->frame_poses.size() <= local_mapper->mpMaxLocalMapSize
        "Local map size should not exceed mpMaxLocalMapSize"

    // Assertion 3 (optional): No hanging state
    // Verify the controller is fully shut down (no joinable threads remain)

    RETURN 0   // Success
```

The following will be added to `CMakeLists.txt` to build the integration test binary.
```cmake
add_executable(basalt_slam src/basalt_slam.cpp)
target_link_libraries(basalt_slam basalt basalt::cli11)
```

**Key architectural differences from `vio.cpp` summarised:**

| Aspect | `vio.cpp` (reference) | `basalt_slam.cpp` (integration test) |
|---|---|---|
| Pipeline wiring | Manual: create `opt_flow_ptr`, `vio`, wire queues explicitly | `Controller` encapsulates all wiring in `initialize()` |
| Data ingestion | `opt_flow_ptr->input_queue.push(data)`, `vio->imu_data_queue.push(data)` | `controller->GrabImage(data)`, `controller->GrabIMU(data)` |
| LocalMapper | Not present | Created by `Controller` in `Controller::initialize`|
| Pose output | Explicit `out_state_queue` + manual thread `t4` | `Controller::process_pose_queue_loop()` (internal thread T4) |
| Visualisation | Pangolin GUI with `out_vis_queue`, draw functions, event loop | None — headless test executable |
| Shutdown | Manual: `vio->maybe_join()`, `drain_input_queues()`, join each thread | `controller->Stop()` — cascade sentinel handles all threads |
| End-of-stream | `feed_images` pushes `nullptr` to `opt_flow_ptr->input_queue` | `Controller::Stop()` pushes `nullptr` — feed threads do **not** push sentinels |
| Post-run | Trajectory saving, alignment, RMSE computation | Assertions on `local_mapper_->frame_poses` |

#### 4.9.5 Eigen Alignment and TBB Containers — The `MatchData` Issue

##### 4.9.5.1 Prerequisites: SIMD, Alignment, and Eigen

Modern CPUs accelerate floating-point arithmetic through SIMD (Single Instruction, Multiple Data) instruction sets. These instructions operate on small, fixed-size vectors of data loaded from memory in a single cycle, but they impose strict **alignment** constraints on the memory addresses they read from:

| Instruction Set | Register Width | Required Alignment |
|-----------------|---------------|--------------------|
| SSE / NEON      | 128-bit       | 16 bytes           |
| AVX             | 256-bit       | 32 bytes           |
| AVX-512         | 512-bit       | 64 bytes           |

An address is *N*-byte aligned when `address % N == 0`. Loading a 256-bit (32-byte) AVX register from an address that is only 16-byte aligned triggers undefined behaviour — on some hardware a silent performance penalty, on others a hard fault.

**Eigen** is a C++ template library for linear algebra that maps its fixed-size matrix and vector types directly onto SIMD registers. When Eigen detects AVX support at compile time (e.g. via `-march=native`), it sets the internal macro `EIGEN_MAX_STATIC_ALIGN_BYTES` and declares fixed-size vectorisable types with the corresponding `alignof`:

| Eigen Type | `sizeof` | `alignof` (AVX) | `alignof` (AVX-512) | Fixed-Size Vectorisable? |
|---|---|---|---|---|
| `Vector2d` | 16 | 16 | 16 | Yes (SSE) |
| `Vector3d` | 24 | 8 | 8 | No (odd size) |
| `Vector4d` | 32 | 32 | 32 | Yes (AVX) |
| `Quaterniond` | 32 | 32 | 32 | Yes (AVX) |
| `Matrix2d` | 32 | 32 | 32 | Yes (AVX) |
| `Matrix4d` | 128 | 64 | 64 | Yes (AVX-512) |
| `AffineCompact2f` | 24 | 4 | 4 | No |

A type is *over-aligned* when `alignof(T) > __STDCPP_DEFAULT_NEW_ALIGNMENT__` (typically 16 on x86-64 Linux). For AVX builds, `Quaterniond` and `Vector4d` are over-aligned at 32 bytes.

**Sophus** wraps Eigen types to represent Lie groups. `Sophus::SO3d` stores an `Eigen::Quaterniond`; `Sophus::SE3d` stores an `SO3d` plus a `Vector3d`. Therefore **any struct containing `Sophus::SE3d` inherits the 32-byte alignment requirement** of its internal `Quaterniond`.

Eigen's documentation ([Explanation of the assertion on unaligned arrays](https://libeigen.gitlab.io/eigen/docs-nightly/group__TopicUnalignedArrayAssert.html)) enumerates four causes of alignment violations:

1. **Struct/class member** — a class containing a fixed-size vectorisable member is heap-allocated via `operator new`, which may not honour the type's alignment. Fix: add `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` to the class.
2. **STL containers** — node-based containers (`std::map`, `std::unordered_map`) allocate nodes internally; the default allocator may not respect the value type's alignment. Fix: use `Eigen::aligned_allocator`.
3. **Pass by value** — passing a fixed-size vectorisable type by value may violate alignment on the callee's stack frame. Fix: pass by const reference.
4. **Compiler stack alignment** — some compilers (old GCC on Windows) assume only 16-byte stack alignment. Fix: compiler flags or `alignas`.

In C++17 (which Basalt requires — see `CMakeLists.txt:87`), `std::allocator<T>::allocate()` for over-aligned types calls `::operator new(size, std::align_val_t(alignof(T)))`, which correctly handles over-alignment. This means **STL containers with the default allocator are safe in C++17** for over-aligned types.

##### 4.9.5.2 Prerequisites: How TBB `concurrent_unordered_map` Stores Values

Intel TBB's `concurrent_unordered_map` is a lock-free hash map built on a split-ordered list. Internally, each key-value pair is stored in a heap-allocated **node**. The relevant class hierarchy from [oneTBB `_concurrent_unordered_base.h`](https://github.com/uxlfoundation/oneTBB/blob/master/include/oneapi/tbb/detail/_concurrent_unordered_base.h) is:

```cpp
// Base node for the split-ordered linked list
class list_node {                              // sizeof = 16
    std::atomic<list_node*> my_next;           // offset  0: 8 bytes
    size_t                  my_order_key;       // offset  8: 8 bytes
};

// Derived node that holds the actual key-value pair
template <typename ValueType, typename SokeyType>
class value_node : public list_node {
    union {
        ValueType my_value;                    // offset 16...or more with padding
    };
};
```

The user provides an allocator for the *value type* (e.g. `Eigen::aligned_allocator<std::pair<const Key, Value>>`). TBB **rebinds** this allocator to `value_node`:

```cpp
using value_node_allocator_type =
    typename allocator_traits_type::template rebind_alloc<value_node_type>;
```

Node allocation in `create_node()` then uses the rebound allocator:

```cpp
value_node_ptr new_node = value_node_allocator_traits::allocate(allocator, 1);
```

The C++ compiler propagates the alignment of `ValueType` to `value_node` (because a derived class's alignment is `max(alignof(base), alignof(members))`). So `alignof(value_node)` equals `alignof(ValueType)` when the value is over-aligned. The compiler also inserts padding between `list_node` and the `my_value` union member to satisfy this alignment:

```
value_node layout (when ValueType has alignof = 32):

Offset  0: [my_next]       8 bytes   ← list_node base
Offset  8: [my_order_key]  8 bytes   ← list_node base
Offset 16: [PADDING]      16 bytes   ← compiler-inserted
Offset 32: [my_value]      ...       ← ValueType starts here (32-byte aligned relative to node base)
```

This means the `ValueType` sits at **offset 32** within the node. For the value to actually be 32-byte aligned in absolute terms, the **node itself** must be allocated at a 32-byte aligned address. This is where the allocator becomes critical.

##### 4.9.5.3 The Root Cause: `Eigen::aligned_allocator` Uses a Global Alignment Constant

`Eigen::aligned_allocator<T>::allocate()` does **not** use `alignof(T)`. It calls:

```cpp
pointer allocate(size_type num, const void* = 0) {
    return static_cast<pointer>(internal::aligned_malloc(num * sizeof(T)));
}
```

And `internal::aligned_malloc()` aligns to `EIGEN_DEFAULT_ALIGN_BYTES` — a **compile-time global constant** determined by SIMD feature detection:

| Architecture | `EIGEN_DEFAULT_ALIGN_BYTES` |
|---|---|
| SSE only | 16 |
| AVX | 32 |
| AVX-512 | 64 |

This is **not** the same as `alignof(T)`. It is a single value for the entire translation unit, regardless of what type `T` the allocator is instantiated for.

The alignment chain works correctly **only when** `EIGEN_DEFAULT_ALIGN_BYTES >= alignof(value_node)`:

| Scenario | `EIGEN_DEFAULT_ALIGN_BYTES` | `alignof(value_node)` | Offset of `my_value` | Effective alignment | Result |
|---|---|---|---|---|---|
| AVX-512 system | 64 | 32 | 32 | `(64n + 32) % 32 = 0` | **OK** |
| AVX system | 32 | 32 | 32 | `(32n + 32) % 32 = 0` | **OK** |
| SSE system compiling with `-march=native` on AVX hardware | 16 | 32 | 32 | `(16n + 32) % 32 = ?` | **FAILS** when `n` is odd |
| Mixed flags across TUs | varies | 32 | 32 | unpredictable | **FAILS** |

The last two scenarios are the dangerous ones. They occur when:
- The system's Eigen package (e.g. `/usr/include/eigen3/`) detects different SIMD capabilities than the project's bundled Eigen.
- TBB was pre-compiled without AVX, so its internal allocator paths use `malloc` (which guarantees only 16-byte alignment on glibc x86-64).
- Different translation units are compiled with inconsistent `-march` flags.

By contrast, C++17's `std::allocator<T>::allocate()` uses `alignof(T)` directly via `::operator new(size, std::align_val_t(alignof(T)))`. This is **per-type**, not global, making it inherently safer.

##### 4.9.5.4 The Crash in Basalt: `MatchData` in `feature_matches`

The `MatchData` struct in `common_types.h` contains `Sophus::SE3d T_i_j`, which contains `Eigen::Quaterniond` (`alignof = 32` on AVX):

```cpp
struct MatchData {
    Sophus::SE3d T_i_j;                                   // alignof = 32
    std::vector<std::pair<FeatureId, FeatureId>> matches;  // alignof = 8
    std::vector<std::pair<FeatureId, FeatureId>> inliers;  // alignof = 8
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```

The `Matches` typedef originally stored `MatchData` **by value** inside a `tbb::concurrent_unordered_map`:

```cpp
// ORIGINAL (before fix)
using Matches = tbb::concurrent_unordered_map<
    std::pair<TimeCamId, TimeCamId>, MatchData,
    std::hash<std::pair<TimeCamId, TimeCamId>>,
    std::equal_to<std::pair<TimeCamId, TimeCamId>>,
    Eigen::aligned_allocator<
        std::pair<const std::pair<TimeCamId, TimeCamId>, MatchData>>>;
```

At `nfr_mapper.cpp:565`, the `operator[]` call triggers TBB's internal `create_node()`, which default-constructs a `MatchData` inside the node. If the node's memory is not 32-byte aligned, the `Quaterniond` constructor inside `SE3d` fires Eigen's alignment assertion:

```
basalt_slam: /usr/include/eigen3/Eigen/src/Core/DenseStorage.h:128: Eigen::internal::plain_array<T, Size, MatrixOrArrayOptions, 32>::plain_array() [with T = double; int Size = 4; int MatrixOrArrayOptions = 0]: Assertion `(internal::UIntPtr(eigen_unaligned_array_assert_workaround_gcc47(array)) & (31)) == 0 && "this assertion is explained here: " "http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html" " **** READ THIS WEB PAGE !!! ****"' failed.

Thread 2 "basalt_slam" received signal SIGABRT, Aborted.
[Switching to Thread 0x7fffebbff640 (LWP 16350)]
0x00007ffff72419fc in pthread_kill () from /lib/x86_64-linux-gnu/libc.so.6
(gdb) bt
#0 0x00007ffff72419fc in pthread_kill () from /lib/x86_64-linux-gnu/libc.so.6
#1 0x00007ffff71ed476 in raise () from /lib/x86_64-linux-gnu/libc.so.6
#2 0x00007ffff71d37f3 in abort () from /lib/x86_64-linux-gnu/libc.so.6
#3 0x00007ffff71d371b in ?? () from /lib/x86_64-linux-gnu/libc.so.6
#4 0x00007ffff71e4e96 in __assert_fail () from /lib/x86_64-linux-gnu/libc.so.6
#5 0x00007ffff79b009e in Eigen::internal::plain_array<double, 4, 0, 32>::plain_array (this=<optimized out>)
at /usr/include/eigen3/Eigen/src/Core/DenseStorage.h:128
#6 Eigen::DenseStorage<double, 4, 4, 1, 0>::DenseStorage (this=<optimized out>) at /usr/include/eigen3/Eigen/src/Core/DenseStorage.h:211
#7 Eigen::PlainObjectBase<Eigen::Matrix<double, 4, 1, 0, 4, 1> >::PlainObjectBase (this=<optimized out>)
at /usr/include/eigen3/Eigen/src/Core/PlainObjectBase.h:476
#8 Eigen::Matrix<double, 4, 1, 0, 4, 1>::Matrix (w=<optimized out>, z=<optimized out>, y=<optimized out>, x=<optimized out>, this=<optimized out>)
at /usr/include/eigen3/Eigen/src/Core/Matrix.h:402
#9 Eigen::Quaternion<double, 0>::Quaternion (z=<optimized out>, y=<optimized out>, x=<optimized out>, w=<optimized out>, this=<optimized out>)
at /usr/include/eigen3/Eigen/src/Geometry/Quaternion.h:297
#10 Sophus::SO3<double, 0>::SO3 (this=<optimized out>) at /ws/ros_ws/src/slam/ext/basalt/thirdparty/basalt-headers/thirdparty/Sophus/sophus/so3.hpp:499
#11 Sophus::SE3<double, 0>::SE3 (this=<optimized out>) at /ws/ros_ws/src/slam/ext/basalt/thirdparty/basalt-headers/thirdparty/Sophus/sophus/se3.hpp:1050
#12 0x00007ffff79bd8cc in basalt::MatchData::MatchData (this=0x7fff54001c30) at /ws/ros_ws/src/slam/ext/basalt/include/basalt/utils/common_types.h:117
#13 std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&, 0ul>(std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>&, std::tuple<>&, std::_Index_tuple<0ul>, std::_Index_tuple<>) (
__tuple2=<synthetic pointer>empty std::tuple, __tuple1=..., this=0x7fff54001c10) at /usr/include/c++/11/tuple:1820
#14 std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>(std::piecewise_construct_t, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>, std::tuple<>) (__second=..., __first=..., this=0x7fff54001c10)
at /usr/include/c++/11/tuple:1809
#15 __gnu_cxx::new_allocator<tbb::detail::d1::value_node<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, unsigned long> >::construct<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>, std::tuple<> >(std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>*, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>&&, std::tuple<>&&) (__p=0x7fff54001c10, this=<optimized out>)
at /usr/include/c++/11/ext/new_allocator.h:162
#16 std::allocator_traits<Eigen::aligned_allocator<tbb::detail::d1::value_node<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, unsigned long> > >::_S_construct<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>, std::tuple<> >(Eigen::aligned_allocator<tbb::detail::d1::value_node<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, unsigned long> >&, std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>*, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>&&, std::tuple<>&&) (__p=0x7fff54001c10, __a=...)
at /usr/include/c++/11/bits/alloc_traits.h:251
#17 std::allocator_traits<Eigen::aligned_allocator<tbb::detail::d1::value_node<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, unsigned long> > >::construct<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>, std::tuple<> >(Eigen::aligned_allocator<tbb::detail::d1::value_node<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>, unsigned long> >&, std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData>*, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>&&, std::tuple<>&&) (__p=0x7fff54001c10, __a=...)
at /usr/include/c++/11/bits/alloc_traits.h:364
#18 tbb::detail::d1::concurrent_unordered_base<tbb::detail::d1::concurrent_unordered_map_traits<std::pair<basalt::TimeCamId, basalt::TimeCamId>, basalt::MatchData, std::hash<std::pair<basalt::TimeCamId, basalt::TimeCamId> >, std::equal_to<std::pair<basalt::TimeCamId, basalt::TimeCamId> >, Eigen::aligned_allocator<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData> >, false> >::create_node<std::piecewise_construct_t const&, std::tuple<std:--Type <RET> for more, q to quit, c to continue without paging--
:pair<basalt::TimeCamId, basalt::TimeCamId>&&>, std::tuple<> >(unsigned long, std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>&&, std::tuple<>&&) (this=0x555555a84dd8, order_key=0) at /usr/include/oneapi/tbb/detail/_concurrent_unordered_base.h:1132
#19 tbb::detail::d1::concurrent_unordered_base<tbb::detail::d1::concurrent_unordered_map_traits<std::pair<basalt::TimeCamId, basalt::TimeCamId>, basalt::MatchData, std::hash<std::pair<basalt::TimeCamId, basalt::TimeCamId> >, std::equal_to<std::pair<basalt::TimeCamId, basalt::TimeCamId> >, Eigen::aligned_allocator<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData> >, false> >::emplace<std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>, std::tuple<> >(std::piecewise_construct_t const&, std::tuple<std::pair<basalt::TimeCamId, basalt::TimeCamId>&&>&&, std::tuple<>&&) (this=0x555555a84dd8) at /usr/include/oneapi/tbb/detail/_concurrent_unordered_base.h:470
^[[A#20 tbb::detail::d1::concurrent_unordered_map<std::pair<basalt::TimeCamId, basalt::TimeCamId>, basalt::MatchData, std::hash<std::pair<basalt::TimeCamId, basalt::TimeCamId> >, std::equal_to<std::pair<basalt::TimeCamId, basalt::TimeCamId> >, Eigen::aligned_allocator<std::pair<std::pair<basalt::TimeCamId, basalt::TimeCamId> const, basalt::MatchData> > >::operator[] (this=this@entry=0x555555a84dd8, key=...) at /usr/include/oneapi/tbb/concurrent_unordered_map.h:99
#21 0x00007ffff799e23e in basalt::NfrMapper::match_stereo (this=this@entry=0x555555a848a0)
at /ws/ros_ws/src/slam/ext/basalt/src/vi_estimator/nfr_mapper.cpp:565
#22 0x00007ffff7ad1626 in basalt::LocalMapper::MapLocally (this=<optimized out>) at /ws/ros_ws/src/slam/ext/basalt/src/vi_estimator/local_mapper.cpp:81
```

The backtrace confirms the crash path:

```
#10 Sophus::SO3<double>::SO3()          ← identity quaternion constructor
#11 Sophus::SE3<double>::SE3()          ← default SE3 constructor
#12 basalt::MatchData::MatchData()      ← default MatchData constructor
 ...
#18 tbb::concurrent_unordered_base::create_node()  ← TBB allocates the node
#20 tbb::concurrent_unordered_map::operator[]()     ← triggered by subscript
#21 basalt::NfrMapper::match_stereo()   ← nfr_mapper.cpp:565
#22 basalt::LocalMapper::MapLocally()   ← local_mapper.cpp:81
```

The crash was observed in the `LocalMapper`'s mapping thread (`std::thread mpLocalMappingThread`). However, the same code path exists in the original offline `NfrMapper::match_stereo()` and `NfrMapper::match_all()` — the bug is latent in upstream Basalt but was not triggered there because:

1. The offline mapper (`basalt_mapper`) runs the code path infrequently.
2. On developer hardware, `EIGEN_DEFAULT_ALIGN_BYTES` may have matched or exceeded 32, masking the issue.
3. Release builds with `-DNDEBUG` disable Eigen's alignment assertions, turning the crash into silent undefined behaviour (misaligned SIMD loads producing subtly wrong results).

##### 4.9.5.5 Which Types Are Affected?

Not all Eigen types in Basalt containers are at risk. Only types with `alignof > 16` (i.e. over-aligned beyond `__STDCPP_DEFAULT_NEW_ALIGNMENT__`) are affected:

| Container | Value Type | `alignof` | Over-Aligned? | At Risk? |
|---|---|---|---|---|
| `Matches` (`feature_matches`) | `MatchData` (via `SE3d`) | 32 | Yes | Yes |
| `Cameras` | `Camera` (via `SE3d`) | 32 | Yes | No — uses `std::map` with `Eigen::aligned_allocator` |
| `Corners` (`feature_corners`) | `KeypointsData` | 8 | No | No |
| `Landmarks` | `Landmark` (`Vector3d`) | 8 | No | No |
| Optical flow maps | `AffineCompact2f` | 4 | No | No |

`KeypointsData` is safe despite containing `std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>>` and `Eigen::aligned_vector<Vector4d>` because these are `std::vector` members that manage their own heap storage independently. The `KeypointsData` struct itself holds only vector metadata (three pointers per vector) and has `alignof(KeypointsData) = 8`.

`Landmark` is safe because `Eigen::Vector3d` is 24 bytes with `alignof = 8` — it is not a power-of-two size, so Eigen does not try to vectorise it with AVX.

##### 4.9.5.6 Architecture-Dependent Behaviour

This issue manifests differently across architectures and build configurations:

**x86-64 with AVX-512 (e.g. Intel Xeon Skylake-SP, Ice Lake):**
- `EIGEN_DEFAULT_ALIGN_BYTES = 64`, `alignof(Quaterniond) = 32`.
- `Eigen::aligned_allocator` returns 64-byte-aligned memory.
- Node base is 64-byte aligned; `my_value` at offset 32 → `64n + 32` → always 32-byte aligned.
- **Bug is masked.** The code works by accident.

**x86-64 with AVX (e.g. Intel Haswell, AMD Zen):**
- `EIGEN_DEFAULT_ALIGN_BYTES = 32`, `alignof(Quaterniond) = 32`.
- `Eigen::aligned_allocator` returns 32-byte-aligned memory.
- Node base is 32-byte aligned; `my_value` at offset 32 → `32n + 32 = 32(n+1)` → always 32-byte aligned.
- **Bug is masked.** The alignment happens to work.

**x86-64 with SSE only, compiled with `-march=native` on AVX hardware:**
- `EIGEN_DEFAULT_ALIGN_BYTES = 16` (SSE detection), but `alignof(Quaterniond) = 32` (AVX instructions enabled by `-march=native`).
- `Eigen::aligned_allocator` returns only 16-byte-aligned memory.
- Node base at `16n`; `my_value` at offset 32 → `16n + 32`. When `n` is odd: `16(2k+1) + 32 = 32k + 48`, and `48 % 32 = 16` → **misaligned. Crash.**

**Mixed compilation (e.g. system TBB pre-compiled without AVX + project compiled with AVX):**
- TBB's internal allocator may use `malloc` (16-byte aligned on glibc x86-64).
- Even if `Eigen::aligned_allocator` is specified, TBB may delegate to its own `scalable_malloc` for some node types.
- **Unpredictable failures** depending on heap layout.

**ARM (e.g. Apple M-series, Jetson):**
- NEON requires 16-byte alignment; `alignof(Quaterniond) = 16`.
- Standard `malloc` on ARM returns 16-byte-aligned memory.
- **No issue.** The types are not over-aligned relative to the platform's default.

##### 4.9.5.7 The Fix: Pointer Indirection via `std::shared_ptr<MatchData>`

The fix decouples the `MatchData` object's memory from TBB's internal node layout by storing a pointer instead of the value:

```cpp
// FIXED
using Matches = tbb::concurrent_unordered_map<
    std::pair<TimeCamId, TimeCamId>, std::shared_ptr<MatchData>,
    std::hash<std::pair<TimeCamId, TimeCamId>>>;
```

**Why this works:**

1. `alignof(std::shared_ptr<MatchData>) = 8`. A `shared_ptr` is just two pointers (16 bytes on x86-64), with no over-alignment requirement. TBB's `value_node` now only needs 8-byte alignment for the value — trivially satisfied by any allocator.

2. The `MatchData` object is allocated on the heap separately via `std::allocate_shared<MatchData>(Eigen::aligned_allocator<MatchData>{}, ...)`. This calls `Eigen::aligned_allocator<MatchData>::allocate()`, which returns memory aligned to `EIGEN_DEFAULT_ALIGN_BYTES`. Critically, the `MatchData` is the **only** object at this address — there is no TBB node header shifting it to an unaligned offset.

3. The `Eigen::aligned_allocator` is no longer needed on the `Matches` typedef itself (since `shared_ptr` has no alignment requirement), simplifying the type.

At every insertion site, `MatchData` is wrapped:

```cpp
// Before (crashed):
feature_matches[std::make_pair(tcid1, tcid2)] = md;

// After (safe):
feature_matches[std::make_pair(tcid1, tcid2)] =
    std::allocate_shared<MatchData>(Eigen::aligned_allocator<MatchData>{}, md);
```

All read sites change `.` to `->`:

```cpp
// Before:
const MatchData& matchData = iter.second;

// After:
const MatchData& matchData = *iter.second;
```

The same approach is applied to `LocalMapper::mpLatestKeyframesMatches` for consistency, so that copies between `feature_matches` and `mpLatestKeyframesMatches` are pointer copies (no `MatchData` duplication, no re-allocation):

```cpp
// local_mapper.h
std::unordered_map<std::pair<TimeCamId, TimeCamId>,
                   std::shared_ptr<MatchData>,
                   std::hash<std::pair<TimeCamId, TimeCamId>>>
    mpLatestKeyframesMatches;
```

**Trade-offs:**

| Aspect | Before (by value) | After (`shared_ptr`) |
|---|---|---|
| Alignment safety | Depends on allocator + TBB internals | Guaranteed by `allocate_shared` |
| Memory layout | `MatchData` inline in node (cache-friendly) | One extra heap allocation + indirection |
| Copy semantics | Deep copy on insert/iteration | Pointer copy (cheaper) |
| Thread safety | TBB map is lock-free; value is owned | `shared_ptr` ref-count is atomic (thread-safe) |
| Code changes | — | `~22 sites`: `.` → `->`, wrap insertions |

The extra indirection is negligible in practice: `MatchData` is accessed infrequently (once per match pair per frame), and the pointer dereference cost is dwarfed by the descriptor matching and geometric verification that produce the data.

---

## 5. Key Classes & Interfaces

### 5.1 Test Infrastructure

| Class/Function | Header | Purpose |
|---|---|---|
| `TEST(Suite, Name)` | `gtest/gtest.h` | Define a test case |
| `EXPECT_TRUE/EQ/LE/NEAR` | `gtest/gtest.h` | Non-fatal assertions |
| `ASSERT_TRUE/EQ` | `gtest/gtest.h` | Fatal assertions |
| `test_jacobian<>(name, Ja, func, x0)` | `test_utils.h` | Central finite-difference Jacobian checker |
| `TestConstants<Scalar>` | `test_utils.h` | Per-type epsilon and tolerance constants |

### 5.2 Data Structures Under Test

| Class | Header | Tests |
|---|---|---|
| `UnionFind` | `include/basalt/utils/union_find.h` | `test_local_mapper` |
| `TrackBuilder` | `include/basalt/utils/tracks.h` | `test_local_mapper` |
| `HashBowStl<256>` | `include/basalt/hash_bow/hash_bow.h` | `test_local_mapper` |
| `LandmarkDatabase<double>` | `include/basalt/vi_estimator/landmark_database.h` | `test_linearization`, `test_local_mapper` |
| `LocalMapper` | `include/basalt/vi_estimator/local_mapper.h` | `basalt_slam` |
| `Controller` | `include/basalt/controller.h` | `basalt_slam` |
| `IntegratedImuMeasurement<double>` | `thirdparty/.../preintegration.h` | `test_vio` |
| `PoseStateWithLin<double>` | `include/basalt/utils/imu_types.h` | `test_linearization`, `test_vio` |
| `NfrMapper` | `include/basalt/vi_estimator/nfr_mapper.h` | `test_nfr` (indirectly) |
| `MargHelper<double>` | `include/basalt/vi_estimator/marg_helper.h` | `test_qr` |
| `LinearizationBase<Scalar, POSE_SIZE>` | `include/basalt/linearization/linearization_base.hpp` | `test_linearization` |
| `OpticalFlowPatch<double, Pattern52>` | `include/basalt/optical_flow/patch.h` | `test_patch` |
| `SplineOptimization<5, double>` | `include/basalt/optimization/spline_optimize.h` | `test_spline_opt` |

### 5.3 Residual Functions Under Test

| Function | Header | Signature | Tests |
|---|---|---|---|
| `relPoseError` | `include/basalt/utils/nfr.h` | `(T_i_j, T_w_i, T_w_j, *Ji, *Jj) → Vec6d` | `test_nfr` |
| `absPositionError` | `include/basalt/utils/nfr.h` | `(T_w_i, pos, *J) → Vec3d` | `test_nfr` |
| `yawError` | `include/basalt/utils/nfr.h` | `(T_w_i, yaw_dir_body, *J) → double` | `test_nfr` |
| `rollPitchError` | `include/basalt/utils/nfr.h` | `(T_w_i, R_w_i_meas, *J) → Vec2d` | `test_nfr` |
| `linearizePoint` | `include/basalt/linearization/...` | Point reprojection linearisation | `test_vio` |
| `computeRelPose` | `include/basalt/vi_estimator/sc_ba_base.h` | Relative pose from frame pairs | `test_vio` |

### 5.4 Data Loading (Integration Tests)

| Class/Function | Header | Purpose |
|---|---|---|
| `DatasetIoFactory::getDatasetIo(type)` | `include/basalt/io/dataset_io.h` | Create dataset reader for "euroc", "bag", "kitti" |
| `DatasetIoInterface::read(path)` | `include/basalt/io/dataset_io.h` | Load dataset from disk |
| `VioDataset::get_image_timestamps()` | `include/basalt/io/dataset_io.h` | Get all image timestamps |
| `VioDataset::get_image_data(t_ns)` | `include/basalt/io/dataset_io.h` | Get image data at timestamp |
| `VioDataset::get_gyro_data()` | `include/basalt/io/dataset_io.h` | Get all gyroscope measurements |
| `VioDataset::get_accel_data()` | `include/basalt/io/dataset_io.h` | Get all accelerometer measurements |

### 5.5 Configuration Files for Tests

| File | Path | Usage |
|---|---|---|
| `euroc_config.json` | `data/euroc_config.json` | VIO configuration for EuRoC dataset |
| `euroc_ds_calib.json` | `data/euroc_ds_calib.json` | Double-sphere camera calibration for EuRoC |

---

## References

1. Google Test documentation: https://google.github.io/googletest/
2. Basalt project: https://gitlab.com/VladyslavUsenko/basalt.git
3. `doc/LocalMapper.md` §11.5 — Testing checklist for LocalMapper.
4. `doc/Mapping.md` — Offline NFR mapper pipeline documentation.
5. `doc/Marginalisation.md` — VIO linearisation and marginalisation theory.
