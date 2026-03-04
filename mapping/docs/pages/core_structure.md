# Core: Buffers and State Management {#core_structure}

This page describes how sensor data flows through the system and how the
sliding-window state is managed.

## Data Flow Overview

Sensor data enters through ROS callback threads and is consumed by a single
update thread:

- **Callback threads** (concurrent, mutex-protected):
  - `feedImu` inserts into `imuBuffer_`
  - `feedLidar` inserts into `lidarBuffer_`

- **Update thread** (sequential pipeline per cycle):
  1. `preintegrateIMU` -- drains `imuBuffer_`, produces propagated NavState
  2. `undistortScans` -- drains `lidarBuffer_`, fills `scanBuffer_` with
     motion-corrected scans
  3. `accumulateUndistortedScans` -- drains `scanBuffer_`, produces a merged
     keyframe submap
  4. `trackScanPointsToClusters` -- matches clusters against the new submap
  5. `updateAndOptimizeGraph` -- runs iSAM2, refines all poses and state


## Buffers

The `Buffers` class holds three distinct buffers with different threading models:

| Buffer | Type | Mutex | Filled by | Cleared by |
|--------|------|-------|-----------|------------|
| `imuBuffer_` | `map<double, ImuData>` | `mtxImuBuffer_` | `feedImu` (callback) | `preintegrateIMU` (update) |
| `lidarBuffer_` | `map<double, LidarData>` | `mtxLidarBuffer_` | `feedLidar` (callback) | `undistortScans` (update) |
| `scanBuffer_` | `list<ScanBuffer>` | None | `undistortScans` (update) | `accumulateUndistortedScans` (update) |

### Threading model

- `imuBuffer_` and `lidarBuffer_` are accessed from both callback threads and the
  update thread, protected by separate mutexes.
- `scanBuffer_` has no mutex because it is only accessed within the single update
  thread (`undistortScans` fills it, `accumulateUndistortedScans` drains it).
- `feedImu` / `feedLidar` take a `lock_guard` to insert one entry and release
  immediately.
- The update thread takes a `lock_guard` (or `unique_lock` for early release) when
  bulk-reading and clearing the input buffers.


## Scan Undistortion and Accumulation

Raw LiDAR scans pass through two stages before becoming a keyframe submap:

### Stage 1: Undistortion

1. Compute the total motion since the last keyframe using the propagated IMU state.
2. Derive constant-velocity angular and linear rates.
3. For each buffered scan:
   - Interpolate `kf_T_scan` (scan pose relative to the last keyframe) from the
     velocity model.
   - Apply per-point motion correction using each point's `offset_time`.
   - Voxel-downsample and push to `scanBuffer_`.

Each `ScanBuffer` entry stores the undistorted pointcloud in its own scan frame
together with the relative pose `kf_T_scan`. Points are **not** yet in world
frame.

### Stage 2: Accumulation

1. The most recent scan defines the new keyframe origin.
2. All scans are transformed relative to this origin and merged into a single
   pointcloud.
3. The merged cloud is voxel-downsampled and returned as the new keyframe submap
   (still in keyframe-local frame).


## States

The `States` class manages the sliding window of keyframes and the navigation
state estimates.

### Sliding window data

Three parallel maps keyed by monotonically increasing keyframe index:

| Map | Content |
|-----|---------|
| `keyframeSubmaps_` | `shared_ptr<PointCloud>` -- submap in world frame |
| `keyframePoses_` | `shared_ptr<Pose3>` -- world-frame pose |
| `keyframeTimestamps_` | `double` -- keyframe creation time |

### Navigation state

| Variable | Description |
|----------|-------------|
| `w_X_curr_` | Current propagated or optimized NavState (pose + velocity) |
| `w_X_preint_` | Reference state for IMU preintegration; reset at each keyframe |
| `currBias_` | Latest IMU bias estimate |
| `imu_T_lidar_` | Fixed extrinsic calibration |
| `smootherEstimate_` | Full iSAM2 output (`Values` containing all window variables) |

### Keyframe creation

`States::createKeyframeSubmap` transforms the submap from keyframe-local to world
frame **in-place**, assigns a new index via `keyframeCounter_++`, and stores
the submap, pose, and timestamp.

### Pose updates after optimization

When iSAM2 refines poses, each keyframe submap is corrected via a left-applied
delta:
\f[
  \Delta T = T_{\text{new}} \cdot T_{\text{old}}^{-1},
  \qquad
  \mathbf{p}' = \Delta T \cdot \mathbf{p}
\f]

This avoids re-transforming from scratch and preserves floating-point consistency.

### Keyframe removal

`States::removeKeyframe` erases a keyframe from all three maps. If visualization
collection is enabled, the submap is archived before deletion and can be drained
via `getMarginalizedSubmaps()`.


## System Lifecycle

`MappingSystem::update()` dispatches on `SystemState`:

### Initializing

Waits until the LiDAR buffer spans at least `init_time_window` seconds, then:

1. Average accelerometer readings to find gravity direction.
2. Build a gravity-aligned initial pose.
3. Stack and transform all buffered scans into an initial submap.
4. Seed premature clusters from the initial submap.
5. Calibrate the IMU preintegrator from static noise statistics.
6. Set prior factors for \f$ X_0, V_0, B_0 \f$.

### Tracking

Runs the full pipeline per `update()` call: preintegrate IMU, check keyframe
threshold, undistort scans, accumulate submap, marginalize, track clusters,
optimize, and seed new clusters.
