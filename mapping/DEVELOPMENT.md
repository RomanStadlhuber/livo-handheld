# Development in the `mapping`  Codebase

## Formatting with `pre-commit`

The repository root holds a `.pre-commit-config.yaml` that runs `clang-format` (style: `mapping/.clang-format`)
on all C++ sources in `mapping/`.

Install the git hook **once, on the host**:

```bash
pipx install pre-commit    # or: pip install --user pre-commit
cd livo-handheld && pre-commit install
```

The `mapping-dev` container already has `pre-commit` (see `Dockerfile.mapping`) and shares the hook through the
bind-mounted `.git/` directory, so commits from inside the container run the same check.

> **Do not run `pre-commit install` inside the container.**
> The hook script stores the absolute path of the interpreter it was installed with.
> From the host, that is the `pipx` venv, which does not exist in the container, so the hook falls back to
> `pre-commit` on `PATH`. From the container, it is `/usr/bin/python3`, which exists on the host but without the
> `pre_commit` module, so the hook fails on the host.

Run the check on all files by hand with `pre-commit run --all-files`.

## Documentation

The project uses Doxygen with the [doxygen-awesome-css](https://github.com/jothepro/doxygen-awesome-css)
theme. The documentation is organized into **modules** (Doxygen groups) that
mirror the source tree.

### Building the docs

```bash
# inside the Docker container
cmake --build /ros2_ws/build/mapping --target docs
```

Output is written to `docs/output/html/`. Serve it locally with:

```bash
cd /ros2_ws/src/livo/mapping/docs/output/html && python3 -m http.server 8080
```

The identifiers in parentheses are the Doxygen group IDs used with `@ingroup`.

### Adding a new file to an existing module

Every `.hpp` and `.cpp` file must have `/// @file` and `/// @ingroup <group_id>` at
the very top (before `#pragma once` or `#include`). This makes the file appear under
its module in the Modules tab instead of the (hidden) Files tab:

```cpp
/// @file
/// @ingroup frontend_imu
#pragma once
// ...
```

For `.cpp` files:

```cpp
/// @file
/// @ingroup frontend_imu
#include <mapping/frontend/ImuFrontend.hpp>
// ...
```

### Adding a class or struct to an existing module

Place `/// @ingroup <group_id>` directly above the class or struct definition.
The comment must be attached to the entity (no blank line between them):

```cpp
namespace mapping
{
    /// @ingroup frontend_imu
    /// @brief Processes raw IMU data.
    class ImuFrontend
    {
        // ...
    };
} // namespace mapping
```

This works the same for structs:

```cpp
namespace mapping
{
    /// @ingroup types
    /// @brief IMU measurement data
    struct ImuData
    {
        Eigen::Vector3d acceleration;
        Eigen::Vector3d angular_velocity;
    };
} // namespace mapping
```

### Adding standalone functions to an existing module

Same pattern — attach `/// @ingroup` to the function doc comment:

```cpp
namespace mapping
{
    /// @ingroup helpers
    /// @brief Convert LidarData to Open3D PointCloud.
    inline open3d::geometry::PointCloud Scan2PCD(/* ... */) { /* ... */ }
} // namespace mapping
```

### Creating a new sub-module (e.g. a new frontend)

1. **Define the group** in `docs/groups.dox`:

   ```
   /// @defgroup frontend_visual Visual Frontend
   /// @ingroup frontend
   /// @brief Camera-based feature tracking.
   ```

2. **Tag classes** in the new header with `/// @ingroup frontend_visual`:

   ```cpp
   namespace mapping
   {
       /// @ingroup frontend_visual
       /// @brief Tracks visual features across frames.
       class VisualFrontend { /* ... */ };
   } // namespace mapping
   ```

3. Rebuild the docs — the new group appears under Frontend in the Modules tree.

### Creating a new top-level module

1. **Define the group** in `docs/groups.dox` (no `@ingroup` parent):

   ```
   /// @defgroup planning Planning
   /// @brief Path planning and trajectory generation.
   ```

2. Optionally define sub-groups with `@ingroup planning`.
3. Tag classes/structs/functions with `/// @ingroup planning`.

### Adding documentation pages

Create a Markdown file in `docs/pages/` with a page anchor in the title:

```markdown
# My New Page {#my_new_page}

Content goes here. Use `\f$ x^2 \f$` for inline math
and `\f[ x^2 \f]` for display math.
```

Link it from `docs/pages/mainpage.md`:

```markdown
- @subpage my_new_page -- Short description
```

### Doxygen version

The project uses **Doxygen 1.13.2**, built from source in the Dockerfile
(`Dockerfile.mapping`). The Ubuntu 22.04 apt package ships 1.9.1 which has
several broken features (e.g. `GROUP_NESTED_COMPOUNDS`, missing MathJax v3
support). Do **not** downgrade to the apt version.

### Building the docs without ROS (standalone Docker)

`Dockerfile.doxygen` at the repo root provides a minimal Ubuntu 22.04 image
with only CMake, Doxygen 1.13.2 and Graphviz — no ROS, GTSAM, or Open3D.
It uses `docs/CMakeLists.txt`, a standalone extraction of the main docs
target, so the build follows the same FetchContent + configure_file path.

```bash
# All commands are run from the repository root (livo-handheld/).

# Build the image
docker build -f Dockerfile.doxygen -t livo-docs .

# Generate the docs
docker run --rm -v $(pwd)/mapping:/source livo-docs
```

Output is written to `mapping/docs/output/html/` on the host, identical to
the CMake-based build.

The entry point `docs/build_docs.sh` also works locally without Docker if
Doxygen 1.13+ and CMake are on `PATH`:

```bash
./mapping/docs/build_docs.sh
```

### Important caveats

- **Use `@ingroup` on each entity**, not `@{` / `@}` scope markers. This is
  explicit and works across all Doxygen versions.
- **Use `\f$` for math in `.md` files** (e.g. `\f$ x^2 \f$`). The `@f$` form
  also works in modern Doxygen but `\f$` is preferred for consistency.
- **`docs/output/` is gitignored** — only source files under `docs/` are tracked.

## Parallelizing Work with TBB

Both GTSAM and Open3D use Intel oneTBB internally for parallel work. GTSAM drives
the main SLAM tracking loop via the incremental fixed-lag smoother (`Smoother.cpp`),
while Open3D is used in `BundleAdjustment` for ICP-based intra- and inter-segment
refinement running in a background thread. Because both libraries draw from the same
global TBB thread pool at runtime, running them concurrently without any partitioning
causes oversubscription. Both can attempt to use all available cores simultaneously,
which starves the tracking thread and breaks real-time pose output.

### TBB task arenas

A `tbb::task_arena` is an isolated scheduling context within the global TBB pool. Each
arena has a maximum concurrency cap and a priority. When work is dispatched via
`arena.execute(...)`, TBB limits the number of worker threads serving that arena to its
cap. When multiple arenas have pending work, it schedules workers preferentially toward
higher-priority arenas. Arenas are unaware of each other.
The TBB scheduler is used to schedule thread work by respecting the concurrency and
priority constraints registered at construction time.

### Partitioning in this codebase

Two arenas are created, one per subsystem, where N is `std::thread::hardware_concurrency()`:

- `frontendArena_` in `Smoother`: N/2 threads, normal priority
- `backgroundArena_` in `BundleAdjustment`: N/2 threads, low priority The caps sum to N so there is no
oversubscription. The low priority on the BA arena means TBB workers will always prefer
pending tracking work over pending BA work when choosing the next task to execute.

The arenas are created as `std::unique_ptr<tbb::task_arena>`, as `task_arena`
contains internal `std::atomic` members and is not assignable. A typical setup and
usage pattern:

```cpp
// src/mapping/backend/Smoother.cpp

// create the TBB arena for GTSAM work (normal priority, half of available cores)
frontendArena_ = std::make_unique<tbb::task_arena>(nCPUs / 2, 1);

// wrap GTSAM work in the arena executor so threads get picked accordingly
frontendArena_->execute([&] { smoother_.update(...); });
```

The BA arena uses the same pattern with `tbb::task_arena::priority::low` as a third
constructor argument.

### Affected files

- [include/mapping/backend/Smoother.hpp](include/mapping/backend/Smoother.hpp) and [src/mapping/backend/Smoother.cpp](src/mapping/backend/Smoother.cpp):
  frontend arena, wraps `smoother_.update()` and `smoother_.calculateEstimate()`
- [include/mapping/BundleAdjustment.hpp](include/mapping/BundleAdjustment.hpp) and [src/mapping/BundleAdjustment.cpp](src/mapping/BundleAdjustment.cpp):
  background arena, wraps `refineSegment()` and `alignAllSegments()`

### Warning: matching shared-library names and versions

> **NOTE:** Any TBB-related linker warnings mentioning OpenCV are harmless.
> OpenCV's TBB dependency is not used by this codebase and the two TBB versions
> coexist without symbol conflicts.

For the arenas to partition a single thread pool, GTSAM and Open3D must resolve to
the same oneTBB runtime. Open3D bundles its own copy and loads it first via its
`RPATH`. GTSAM's `libtbb.so.12` dependency then resolves to that already-loaded
instance through soname deduplication rather than pulling in the system copy. The
result is one shared oneTBB runtime for both libraries. This can be verified by
checking that only one `libtbb.so.12.*` path appears in `/proc/<pid>/maps` while
the node is running.

```bash
cat /proc/<pid>/maps | grep tbb
```

The output should show exactly one `libtbb.so.12.*` path. If two distinct paths appear,
GTSAM and Open3D have loaded separate TBB instances and the arenas no longer partition
a shared pool.

## Debugging Instrumentation `ENABLE_DBG_CMP`

To enable logic that computes instrumentation as part of the control flow,
add the following variable when building.

```bash
colcon build --symlink-install --packages-select mapping \
    --cmake-args -DENABLE_DBG_CMP=ON
```

Or, in `CMakeLists.txt`, flip the default of the existing option from `OFF` to `ON`:

```cmake
option(ENABLE_DBG_CMP "enable debug computations (summaries, tracking statistics)" ON)
```


Rebuild without the flag (or with `-DENABLE_DBG_CMP=OFF`) to disable instrumentation.
When adding new diagnostic-only logic, wrap it in
`#ifdef ENABLE_DBG_CMP` so it is excluded from release-style builds.

## Benchmark Build `MAPPING_BENCHMARKING`

The option `MAPPING_BENCHMARKING` (default `OFF`) compiles the benchmarking code in `src/mapping/benchmarking/`:
the run output directory, `TrajectoryRecorder`, the git build info and the keyframe telemetry
(call-point macros in `mapping/benchmarking/telemetry_macros.hpp`). The production binary has none of it.

```bash
colcon build --symlink-install --packages-select mapping \
    --cmake-args -DMAPPING_BENCHMARKING=ON
```

The option is stored in the CMake cache. Switch back with `-DMAPPING_BENCHMARKING=OFF`,
or use separate `--build-base` / `--install-base` folders for benchmark and production builds.

In a benchmark build, bag mode writes one directory per run:

```bash
ros2 run mapping mapper --bag <bag_dir> --config <config.yaml> --output-dir <repo>/benchmark_runs/<name>
```

- `trajectory.tum`: keyframe poses (IMU frame) with absolute bag timestamps, readable by `evo`
- `telemetry.csv`: one row per keyframe that `track()` creates, also for keyframes that lose tracking, see below
- `clusters.csv`: one row per LiDAR cluster factor in the smoother before each update that marginalizes, see `ClusterSpanRow`
- `config.yaml`: the `MappingConfig` used by the run, serialized after loading. It holds the real values incl. defaults for missing keys, not the user file
- `run.yaml`: git commit and dirty flag, build type, inputs, timing and result counters, exit code

The command exits with code 1 if `<dir>` exists and is not empty. In a production build `--output-dir` exits with code 1.
The git commit is read at every build, so build from the main checkout and move it with `git switch --detach <branch>`.
A build inside a worktree folder reports `unknown`. To keep runs inside the repo, put them under `<repo>/benchmark_runs/`. Git ignores this folder.

### Keyframe Telemetry

`telemetry.csv` holds one row per keyframe. The column `t` has the same value as the timestamp in `trajectory.tum`,
so a time on the error curve leads to exactly one telemetry row. Columns without a value are `nan`,
e.g. the IMU and backend columns of a keyframe with `lost_tracking=1`.
The schema and the meaning of each column are in `mapping/benchmarking/KeyframeTelemetry.hpp`.

The only call point is one line in `MappingSystem::track()`:

```cpp
BENCH_KEYFRAME_PROBE(idxKeyframe); // benchmark build only: records this keyframe's telemetry on scope exit
```

The macro creates a `benchmarking::KeyframeProbe`. Its constructor reads the values that the update overwrites,
its destructor runs on every exit of `track()` and collects the rest through the free functions in
`src/mapping/benchmarking/Instrumentation.cpp`. Do not add telemetry code to the SLAM components.

To add a column:

1. Add one line to `KF_TELEMETRY_FIELDS` in `KeyframeTelemetry.hpp`, e.g. `X(Real, my_value)`.
   Types: `Index`, `Flag`, `Real`, `Time`, `Stats` (6 columns `_n`, `_min`, `_max`, `_median`, `_mean`, `_std`).
2. Set `row.my_value` in `Instrumentation.cpp`. A `Stats` value comes from `makeStats(std::vector<double>)`.

The member, the CSV header and the CSV writer follow from the list.

### Evaluation Script

`scripts/evaluate_run.py` joins the trajectory error of a run to its telemetry. It uses its own venv, not the ROS
Python, and runs without a sourced ROS setup. Set up the venv once, from the repository root:

```bash
python3 -m venv mapping/scripts/.venv
mapping/scripts/.venv/bin/pip install -r mapping/scripts/requirements-eval.txt
```

Run it on a run directory and a ground-truth TUM file:

```bash
mapping/scripts/.venv/bin/python mapping/scripts/evaluate_run.py benchmark_runs/<name> <ground_truth.txt>
```

Options: `--t-max-diff` (default 0.05 s, max. timestamp difference to match a ground-truth pose),
`--align-seconds` (default 5 s, SE(3) alignment on the start of the run only, no global alignment),
`--segment` (default 1 s, segment length of the relative error), `--threshold` (default 1 m),
`--gyro-bias-seconds` (default 2 s), `--static-end` (default: detected from the gyro rate), `--gyro-until` (default 14 s).

It writes into `<run dir>/eval/`:

- `errors.csv`: the telemetry with the error columns (`ape_*`, `rpe_*`), one row per keyframe.
  Keyframes without a matched pose (e.g. lost tracking) keep their row with `nan` errors
- `overview.png`: error curves on top, then one panel per stage (IMU, LiDAR frontend, geometry, backend).
  Red lines mark lost tracking, orange lines the recovery keyframe after it
- `gyro_consistency.yaml`: mean gyro gap for the static phase and for `t < --gyro-until`. The gyro gap
  (column `gyro_gap_deg`) is the angle between the estimated rotation from the previous keyframe and the rotation from
  the integrated raw gyro (bias = mean of the first `--gyro-bias-seconds`). The script reads the gyro from
  `input.bag` / `input.topic_imu` in `run.yaml`. If the bag is not readable at that path, the script prints a warning
  and writes the other files without the gyro gap

On stdout it prints the gyro gap means, the first time the absolute error passes `--threshold`, and the telemetry row at that time.
Plot title and summary show the commit, dirty flag and config name from `run.yaml`.
A run directory written from the container belongs to `root`. Give it to your user first
(`sudo chown -R $USER benchmark_runs/<name>`) so the script can create `eval/`.

### Marginalization Analysis Script

`scripts/block_release_analysis.py` reads the detail files of runs (same venv):

```bash
# span distribution and block-release simulation (shed / end at boundary) from clusters.csv
mapping/scripts/.venv/bin/python mapping/scripts/block_release_analysis.py spans benchmark_runs/<name> --out <dir>
# smoother update time per keyframe (t_update_ms): median, p95, max
mapping/scripts/.venv/bin/python mapping/scripts/block_release_analysis.py timing benchmark_runs/<a> [<b> ...]
```

The definitions of the views and of the simulation are in the docstring of the script.

`backend.block_release_size` (benchmark build only, default 0 = off) marginalizes the keyframes in contiguous blocks:
the smoother re-stamps the `X`/`V`/`B` keys of each completed block to its newest keyframe, so one update marginalizes
the whole block. Clusters that lie inside the block (plus the next keyframe) keep their factor and go whole into the
marginal (`lidar_fac_marg_whole`), crossing clusters are rebuilt with a unary factor as before.

The initialization keyframe `0` is not created by `track()`, so it has a pose in `trajectory.tum`
but no telemetry row. This is a known exception to "every `t` in `trajectory.tum` has a telemetry row".
The script reports it as one unmatched pose.

## Profiling with `gperftools`

Inside the docker container install

```bash
apt update && apt install -yq google-perftools libgoogle-perftools-dev graphviz
```

Build using `RelWithDebInfo`

```bash
mkdir build
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

Run the node in bag reader mode with the profiler

```bash
./scripts/gperf.sh --config config/mapping_mid360.yaml --bag /path/to/bag
```

Use `-o` to specify a custom output path (defaults to `/tmp/mapper_profile.prof`)

```bash
./scripts/gperf.sh -o /path/to/profile.prof --config config/mapping_mid360.yaml --bag /path/to/bag
```

Press `CTRL+C` to stop early — the profile data is flushed before the process exits.

### Process and visualize the profiling results

To process the profiling data, install the Go-based `pprof` inside the container
(the Perl-based `google-pprof` from apt is too slow for large debug binaries):

```bash
apt-get install -yq golang
GOBIN=/usr/local/bin go install github.com/google/pprof@v0.0.0-20240727154555-813a5fbdbec8
```

**NOTE:** `@latest` requires Go 1.24+, but Ubuntu 22.04 ships Go 1.18, so we pin an older compatible version.

Convert the profiling data to protobuf format and copy it to the host:

```bash
pprof -proto ./build/mapper /tmp/mapper_profile.prof > /tmp/profile.pb.gz
```

```bash
docker cp mapping-dev:/tmp/profile.pb.gz .
```

Visualize by dropping `profile.pb.gz` into [speedscope](https://www.speedscope.app/),
or with the Go-based pprof web UI:

```bash
pprof -http=:8080 profile.pb.gz
```

### Cleaning up `colcon-clean`

From inside the docker container, you can install (might need pip3 first).
AFAICT, colcon-clean is the ROS2 euqivalent to `catkin_make clean`.

```bash
# may need $ apt install -yq python3-pip
pip install colcon-clean
```

and clean up this package with

```bash
colcon clean packages \
    --base-select \
        build \
        install \
    --packages-select mapping
```
