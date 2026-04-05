# Backend: iSAM2 Management {#backend_isam2}

This page describes how the incremental fixed-lag smoother (iSAM2) is configured,
how factors are managed across keyframes, and how marginalization is handled.

## iSAM2 Configuration

The smoother is initialized in `Smoother::reset` with these settings:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Optimization | Gauss-Newton | Not Dogleg |
| `relinearizeThreshold` | `isam2_relinearize_threshold` (default 0.01) | Frobenius norm; tighter than GTSAM default of 0.1 |
| `relinearizeSkip` | 1 | Check every update call |
| `findUnusedFactorSlots` | `true` | Required by fixed-lag smoother for factor slot reuse |
| Lag window | `sliding_window_size` (default 7 keyframes) | Interpreted as keyframe-index span |


## Graph Variables

Each keyframe \f$ k \f$ introduces three GTSAM variables:

| Symbol | Type | Description |
|--------|------|-------------|
| \f$ X(k) \f$ | `Pose3` | World-frame pose of the IMU at keyframe \f$ k \f$ |
| \f$ V(k) \f$ | `Vector3` | World-frame velocity |
| \f$ B(k) \f$ | `imuBias::ConstantBias` | Accelerometer and gyroscope biases |


## Update Cycle

`Smoother::updateAndOptimizeGraph` runs once per keyframe with this sequence:

### 1. Assemble new factors and values

The `CombinedImuFactor` between \f$ (X_{k-1}, V_{k-1}, B_{k-1}) \f$ and
\f$ (X_k, V_k, B_k) \f$ is appended to the feature factors from the
FeatureManager. Initial values for \f$ X_k, V_k, B_k \f$ come from the IMU-propagated
state.

### 2. Bootstrap (first call only)

On the very first update, stored prior factors for \f$ X_0, V_0, B_0 \f$ from
static initialization are injected alongside the new factors.

### 3. Primary iSAM2 update

```cpp
smoother_.update(newFactors, newValues, timestamps, factorsToRemove);
```

This single call inserts variables into the Bayes tree, adds/removes factors, and
performs one Gauss-Newton iteration.

### 4. Additional GN iterations

```cpp
for (size_t i = 1; i < config.solver_iterations; i++)
    smoother_.update();  // empty call = one more GN step
```

Each empty `update()` call performs one additional GN step without modifying the
graph structure. The default `solver_iterations = 2` means one substantive call
plus one refinement pass.

### 5. State extraction

After optimization, `smoother_.calculateEstimate()` produces the refined values.
The system updates:
- Current pose + velocity from \f$ X_k, V_k \f$
- IMU bias from \f$ B_k \f$
- Preintegration reference state (for the next integration cycle)
- All keyframe submap poses via left-composed deltas:
  \f[
    \dT = \mtx{T}_{\text{new}} \cdot \mtx{T}_{\text{old}}^{-1},
    \qquad
    \bvec{p}_{\text{world}}' = \dT \cdot \bvec{p}_{\text{world}}
  \f]


## Marginalization

`MappingSystem::marginalizeKeyframesOutsideSlidingWindow` runs **before** the
smoother update to circumvent the blanket variable fixing that iSAM2 does in
[`iSAM2::marginalizeLeaves`](https://gtsam.org/doxygen/a04340.html#a321fb6f90eb0035ef8e5bb383f8da4a2).


### The Problem with iSAM2's Marginalization

> Marginalization leaves a linear approximation of the marginal in the system,
> and the linearization points of any variables involved in this linear marginal
> become fixed.
> 
> The set fixed variables will include any key involved with the
> marginalized variables in the original factors, and possibly additional ones
> due to fill-in.

which means that when a varialble \f$ X_k \f$ becomes marginalized
during `IncrementalFixedLagSmoother::update`, all other variables
in the markov blanket (i.e. associated through LiDAR factors) **will
be locked**.

While this design decision makes sense for long runing factor graphs
(e.g. when not using a sliding window), this is detrimental to the
odometry process.
Due to the fact that LiDAR cluster factors often associate with
(almost) the entire sliding window, marginalizing this way would mean
that that essentially the entire window would become locked, and new
variables will not have time to mature over multiple optimizer passes.

### The Workaround used in this project.

The workaround is to update the factors by separating out the variable that will be marginalized in the *next* `update()` call and creating a
custom marginalization-factor that is a `NonLinearContainerFactor`
around a `JacobianFactor` with fixed nonlinear residual \f$ r_{j}
(X_{k}) \f$ which is created from 
`mapping:PointToPlaneFactor::createMarginalizationFactor()`, which is
one prior for each cluster the variable is associated with.
These will then be amalgamated into a Hessian factor by iSAM2
internally and serve to provide a stronger prior to the
marginalization process itself.

Removing the variable associations from the cluster but *not* creating
the marginalization priors would lead to a weak marginalization which
makes drift worse, as the IMU preintegration factor connected to
\f$ X_{k} \f$ would then serve as the only prior information.


### The Marginalization Process


For each keyframe older than `idxKeyframe - sliding_window_size`:

1. **Compute Markov blanket** -- Collect \f$ X(k) \f$ for all \f$ k \f$ from the
   marginalized index to the current keyframe. This provides the linearization
   point for the marginalization factor.

2. **Create marginalization priors** -- For each cluster that references the
   marginalized keyframe, `PointToPlaneFactor::createMarginalizationFactor`
   produces a `LinearContainerFactor`. This frozen Jacobian prior preserves the
   information from the dropped keyframe on the remaining variables.

3. **Remove keyframe from clusters** -- The oldest point association and thickness
   history entry are removed. If a cluster drops below 3 points it becomes
   `Pruned`; if it was `Idle` it becomes `ShiftedIdle` (needing factor
   reconstruction).

4. **Erase from States** -- The keyframe's submap, pose, and timestamp are
   removed (optionally archived for visualization).

The iSAM2 fixed-lag smoother automatically handles variable elimination for
\f$ X_k, V_k, B_k \f$ when their timestamp falls outside the window, but LiDAR
factors referencing those variables must be pre-processed (dropped or converted to
priors) before that happens.


## Factor Management

The FeatureManager dispatches on cluster state to manage factors:

| Cluster State | Action |
|---------------|--------|
| Premature | Skip -- no factor |
| Tracked | Create new `PointToPlaneFactor` or **replace** existing one (copy-add-remove) |
| Idle | Update plane parameters in-place via shared pointers |
| ShiftedIdle | Replace factor with reduced key set, then transition to Idle |
| Pruned | Remove factor from graph |

**Why copy-add-remove?** GTSAM's Bayes tree does not support re-keying factors
in-place. When a cluster gains or loses a keyframe association, the old factor
must be removed and a new one inserted. This is tracked via `factorsToRemove_`
indices returned alongside the new factor graph.


## Configuration Parameters

```yaml
backend:
  sliding_window_size: 7       # keyframes in the active graph
  init_time_window: 2.0        # seconds of static data for initialization
  solver_iterations: 2         # total iSAM2 update() calls per keyframe
  isam2_relinearize_threshold: 0.01  # Frobenius norm for relinearization
```
