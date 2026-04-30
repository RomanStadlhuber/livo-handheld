# Development in the `mapping`  Codebase

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
