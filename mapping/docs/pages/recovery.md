# Tracking Recovery {#recovery}

When the LiDAR frontend fails to track clusters against a new keyframe,
the system enters `SystemState::Recovery`.
Tracking will fail when all tracking candidates fail the validation stages
(\f$ 6 \sigma \f$ test or plane normal consistency check)
or no valid planar tracks could be found from KNN search in the first
place.

This page describes how a recovery
pose is estimated, how the system state is reset, and how this is
integrated with the remaining system to continue tracking.


## Trigger

Tracking loss is detected in `MappingSystem::track()` when
`LidarFrontend::trackScanPointsToClusters` returns `false`. The system
transitions to `SystemState::Recovery` and the next call to
`MappingSystem::update()` executes the recovery case.


## Recovery State Estimation

`RecoveryFrontend::estimateRecoveryState` produces a full `gtsam::NavState`
(pose + velocity) for the keyframe where tracking was lost. The procedure has
two stages: pose extrapolation followed by ICP refinement.

### 1. Initial pose guess via constant-velocity extrapolation

A reference window of keyframes is selected from the sliding window history,
offset by `reference_lag` keyframes before the failed keyframe:

```
  ... [start] --- ... --- [end] --- (lag) --- [recovery]
       |____ window ____|         |_ lag _|
```

The `se(3)` twist between the two most recent keyframes in the
reference window gives the constant-velocity motion model:

\f[
  \boldsymbol{\xi} =
    \frac{1}{\Delta t_{12}} \;
    \text{Log}\bigl({{}^w\mathbf{T}_{l_1}}^{-1} \; {}^w\mathbf{T}_{l_2}\bigr)
\f]

The initial recovery pose is then extrapolated forward:

\f[
  {}^w\mathbf{T}_{l,\text{init}} =
    {}^w\mathbf{T}_{l_1} \;
    \text{Exp}\bigl(\boldsymbol{\xi} \cdot \Delta t_{1 \to \text{rec}}\bigr)
\f]

### 2. Point-to-plane ICP refinement

A reference point cloud is built by merging and voxel-downsampling all
keyframe submaps inside the reference window. The failed keyframe's submap
serves as the query cloud:

| Parameter | Config field | Default |
|-----------|-------------|---------|
| Reference voxel size | `recovery.voxel_size` | 0.2 m |
| Reference window size | `recovery.reference_window_size` | 10 keyframes |
| Reference lag | `recovery.reference_lag` | 2 keyframes |
| Max ICP iterations | `recovery.icp_iterations` | 50 |
| Max correspondence distance | `recovery.max_correspondence_distance` | 0.05 m |

Open3D's
[point-to-plane ICP](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html#Point-to-point-ICP)
aligns the query against the reference, starting
from the extrapolated pose. The output is the refined recovery pose
\f$ {}^w\mathbf{T}_{l,\text{ICP}} \f$.

#### Some notes about the Configuration

##### Maximum Correspondence Distance

While the ICP method is initialized from extrapolating the latest
reference pose, it is **not feature based** and uses the points proximity to
pick point-to-plane correspondences.
This means that while a looser `max_correspondence_distance` might be able to
recover from a bad initialization, this might come at the cost of noisy
correspondences influencing final pose estimate.

##### Lag and Window Size

Instead of running ICP against only the last keyframe submap, a configurable
amount of keyframes is merged together to increase the likelihood of finding
reasonable correspondences.
Picking a sensible window size will depend on the structure of the environment,
but making it close to the backends sliding window size is usually a good
starting point.

Additionally, a lag is used such that a more mature pose is used as reference
pointcloud.
**This is a tradeoff**, where a **higher lag** will lead to **more robust
reference** poses and pointclouds, but also increase the **potential for bad
initialization** from extrapolation that does not fit the true motion (as
tracking is usually lost during high dynamic motion).


### 3. Velocity correction

The pre-tracking velocity \f$ {}^w\mathbf{v} \f$ from the IMU propagation is
still expressed in the predicted (failed) body frame. It is rotated into the
ICP-refined frame:

\f[
  {}^w\mathbf{v}_{\text{rec}} =
    {}^w\mathbf{R}_{\text{ICP}} \;
    {}^w\mathbf{R}_{\text{pred}}^{-1} \;
    {}^w\mathbf{v}_{\text{pred}}
\f]

The final `gtsam::NavState` combines the ICP pose with the corrected velocity.


## Control Flow

1. Tracking lost at keyframe N: `keyframeCounter_` is N + 1, system enters `Recovery`.
2. `estimateRecoveryState()`: extrapolate pose from reference window, refine with point-to-plane ICP, rotate velocity into ICP frame.
3. Reset `FeatureManager`, `States`, `Buffers`, and `Smoother`: only the recovery-keyframe `N`
survives in States.
4. `setPriors(idxKfRecovery = N, w_X_recovery, bPrior)`: anchor the fresh graph at the recovery keyframe.
5. Reset preintegrator, create new clusters from recovery keyframe submap.
6. Transition to `SystemState::Tracking`: next `update()` call resumes normal tracking, next keyframe gets index N + 1.
