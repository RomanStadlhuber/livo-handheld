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

### Important caveats (Doxygen 1.9.1)

- **Use `@ingroup` on each entity**, not `@{` / `@}` scope markers. The scope
  marker approach does not associate classes with groups in Doxygen 1.9.1.
- **Use `\f$` for math in `.md` files**, not `@f$`. The `@f$` form does not
  render correctly in Markdown pages with this Doxygen version.
- **Avoid `\lvert` / `\rvert`** in math — use `\left|` / `\right|` instead
  (MathJax v2 compatibility).
- **`docs/output/` is gitignored** — only source files under `docs/` are tracked.

## Profiling with `gperftools`

Inside the docker container install

```bash
apt update && apt install -yq google-perftools libgoogle-perftools-dev graphviz
```

Build using `RelWithDebInfo`

```bash
colcon build --symlink-install --packages-select mapping --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
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