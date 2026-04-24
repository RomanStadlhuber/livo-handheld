# Improvements to the Mapping Pipeline

Some ideas that I need to try out in order to improve the robustness and accuracy of the system

## Realtime Capability Profiling

Find out how realtime-capable the SLAM is

- What is the average time between keyframes?
- Can it finish all accumulation, tracking & mapping logic before the next keyframe?
- Does it need to finish before the next incoming scan -> otherwise how can state prediction be correct?

## Implement proper, confiugrable & colorized logging

ERROR, WARN, INFO, DEBUG - configurable **per module**

- i.e. the Imu frontend can report DEBUG data, while other modules just emit INFO (see next issue)
- proper coloring for each log level
- wrapping of messages in "::: [LEVEL] {msg} :::" (tabs & bullet points for associated multi-lines)
- optional (MappingSystem-based) timestamping for logs (i.e. the timestamps associated with the current LiDAR / IMU data)

### Module Logging Ideas

#### ImuFrontend

- Number of datapoints used for preintegration in this cycle (i.e. when calling preintegrate), timstamp range of the current cycle.
- Number of entries & timestamp range since last keyframe (as there are multiple preintegrations between keyframes)
- [WARN] log when preintegration buffer is empty

#### Smoother

- [INFO] Number of factors currently in the graph after 1st call to update
  - (maybe set to debug later when everything works)
- [DEBUG] Pose Deltas over all updates (+ additional GTSAM info if necessary)

## Troubleshoot Preintegraiton Divergence

On the HILTI SLAM dataset, the preintegration divered, causing arbitrary rotations of almost ~3.14 [rad].

- Have a look at the logs to find what is indicating the issue
- configure ImuFrontend & Optimzer to DEBUG logs

## Feed back factor indices in the graph from `Smoother::update` for use in `FeatureManager::createAndUpdateFactors`

Currently the FeatureManager needs to scan the graph current NonlinearFactorGraph which is obtained from Smoother beforehand.
This scan is O(N * M) and needs to be done on each iteration of Smoother::createAndUpdateFactors.
It would be better if there were some state sharing structure in place that is able to share the indices of newly added factors in the graph.


When using iSAM2, **factor indices are not reused**, so whenever new factors are added using `IncrementalFixedLagSmoother::update`,
the indices of the new factors in the graph are returned by the `ISAM2Result::getNewFactorIndices`, which correspond to the order of insertion.

Accumulating this information in the state (e.g. making the "cluster to factor" mapping a class member to FeatureManager rather than just
creating and filling it locally) would effectively eliminate the need for this search entirely as the index mapping information gets built
up directly after the factors are created and updated.

There are two prerequisites for this mechanism

- A factor cannot go without an index mapping, otherwise a search needs to be started to eliminate this and all other "orphaned" clusters
- The `isam2::update` result would either be returned by the smoother update implementation and passed to the FeatureManager by the
  MappingSystem or `Smoother::updateAndOptimizeGraph` requires access to the FeatureManager and modifies its state from outside

#### Feeding back the Index Info

I favor the latter implementation because then that's just two backend modules interacting with each other and it's less complicated.
As `Smoother::updateAndOptimizeGraph` already separates the IMU and LiDAR factors in the parameters, it has full control over the
order of insertion.
E.g. when it inserts the IMU factor first, follwed by all the LiDAR factors, it already knows to ignore the first index and report back
all others.

The only missing aspect is that the new LiDAR factor variables don't tell about where they sit in the mapping,
**Update:** I looked it up again and the mapping is actually just (clusterId -> factorIdx).
As there factor class also has a member `PointToPlaneFactor::clusterId_` for this, the info can be easily fed back into the mapping.

As an additional fallback, the `createAndUpdateFactors` class can do an ad-hoc search whenever the std::map for the cluster index mapping
does not contain an idx map for the current cluster id and fill that in on its own.


## Scan-To-Map Registration

Go beyond the local state.
Submaps, once marginalized, could get accumulated into a static map buffer that is continuously mapped against.

This extends the constraint form between poses to unary "relocalization" constraints of the current state against the static map.

- Marginalized Submaps can be accumulated into larger submaps & refined further
- i.e. a static submap could be built on 5-10 [m] sections of the marginalized trajectory submaps
- this submap could then be further refined with outlier points removal

The idea for this is inspired by other mapping systems that effectively use map features that are not part of the state for
trajectory estimation, e.g.

- Fast-LIO & Fast-LIVO
- Open3D-SLAM
- MOLA, the Modular LidAR Localizaiton and Mapping Toolbox (realtime LIO)

## Misceral - Try out Pointcloud colorization with HILTI-SLAM dataset

Is there an RGB camera on the handheld device that I can use for proper colorization?

