# Secondary Bundle Adjusment Thread

Initial work revealed that the LiDAR inertial tracking capabilities are insufficient to
constrain the state ovver larger periods of time that exceed the sliding window by
a large amount.
Specifically, when building a pointcloud map by simply concatenating marginalized submap
pointclouds and revisiting places, offsets in the global pose would become evident.

Therefore, a secondary pose graph optimization (PGO) module - `BundleAdjustment` - is
implemented to build a globally consistent map for the tracking to register to in an
effort to improve the global pose accuracy and mitigate drift when revisiting previously
seen locations.

## Core Idea

After poses and their submap pointclouds have been marginalized from the sliding window
and are no longer actively used for tracking, they are shifted into the secondary pose
graph by `BundleAdjustment::accumulateSubmap`.

The secondary pose graph is loosely coupled and handled by
[Open3Ds Multiway registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html)
pipeline, which optimizes the poses (nodes) using constraints (edges) informed by
the main LiDAR inertial tracking poses and point-to-plane ICP - in a loosey coupled manner.

> **Note:** to reduce the computational load, the number of submaps accepted into the
> secondary graph is limited by configurable translation & orientation thresholds
> see (`BundleAdjustmentConfig`).

## The Submap Lifecycle

Once a submap gets marginalized from the sliding window, its LiDAR-frame pose
\f$ \wTl \f$  is compared against that of the last accumulated submap, and accepted into
the PGO buffer when exceepding a certain pose delta.
These submaps are then waiting to be optimized.

The BA system will await `BundleAdjustment::MIN_ACTIVE_SUBMAPS` before beginning the
processing.
This is for two reasons:
- before building the graph which is to be optimized, the submaps are further aligned
  using ICP to mitigate coarse drift between the active submap pointclouds
- running ICP for sequential and loop closure constraints on multiple active nodes
  greatly reduces the chances of PGO falling into bad local minima

When submaps are fed into the PGO for the first time, and until they are considered
converged, they are referred to as *active submaps*.
Submaps that have been updated by PGO until their poses converged or they reached the
maximum number of iterations will become *frozen*.
Frozen submaps act as static reference nodes for active submaps to do loop closure checks
against and increase the number of constraints beyond just odometry edges.
The very first submap acts as the global anchor (i.e. fixing the identity pose) and is
therefore frozen immediately.

### Building the Pose Graph

Open3Ds global optimization can only truly fix one node.
Therefore, in the first pass, the poses of all other frozen submaps are fixed by
connecting an edge from each to the anchor node, setting as `uncertain=false` (so that
the PGO internally doesn't discard these edges and make the nodes floating) and setting
the information matrix to a very high value.
Ablation experiments revealed that simply connecting all \f$ R \f$ frozen submaps to the
global anchor, instead of doing \f$ R^{2} \f$ connections acts as a sufficient constraint
and greatly reduces the computational load on the optimization process.

After fixing the frozen submaps, the sysem will insert odometry edges (using the initial
ICP refined poses and their associated) between all active submaps in temporal sequence.
Additionally, each submap is checking other nearby submaps (both frozen and active) for
loop closure constraints, of which there will usually be plenty, given there is
sufficient overlap between the pointclouds.
However, to further bound the computational load on the PGO process itself, the number of
loop closures per submap is bounded to `MAX_PGO_LOOP_CLOSURES`, a low number.
While the odometry edges are set as `certain=true`, loop closure edges are uncertain so
that the outer PGO loop has the ability to reject spurious associations that would
otherwise corrupt the system.

All edges, both odometry and loop closures, are informed by ICP and information matrices
based on geometric overlap.
ICP is initialized from the latest known pose (provided either by the LiDAR-IMU tracking
or PGO) and provides relative poses and an overlap score (`fitness`), while the pose
edges information matirx is obtained separately, too based on the geometry.

### Optimizing the Graph

Open3D does pose graph optimization in an inner and outer loop.
The inner loop is informed by the poses and information matrices, building a large
(and continuously growing) system

\f[
    \mtx{J}^{T} \mtx{\Lambda} \mtx{J} \cdot \Delta{\bvec{x}} =
    J^{T} \mtx{\Lambda}  \cdot \bvec{r}
    \text{, }
\f]

where \f$ \mtx{\Lambda} \f$ is the information matrix, \f$ \bvec{r} \f$ the pose errors
in the tangent space.
Jacobians are the standard exponential map linearization and the hessian will be
block-sparse.
The system is then optimized using the Levenberg-Marquardt method with up to \f$ N \f$
steps.
This is one inner iteration.

The outer loop will re-evaluate edges with `uncertain=true` and disable them if they
distort the system.
For details on the algorithm that is used to achieve this, refer to
[(Chou, Zhou and Kotlun, 2015)](https://www.cv-foundation.org/openaccess/content_cvpr_2015/papers/Choi_Robust_Reconstruction_of_2015_CVPR_paper.pdf)
.


## Moving BA to the background

To reduce the computational load on the CPU and make sure there are always cores
available for tracking, we split the entire project into the "tracking" or "foreground"
part and "bundle adjustment" or "background" part.

As both GTSAM and Open3D naturally make use of TBB for the actual optimizer work,
this split can be easily achieved using the libraries
[task_arena](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/task_arena/task_arena_cls.html#_CPPv410task_arena11constraintsj8priority)
API.
This is as easy as creating two separate arenas (that don't need to be aware of each
other), and giving the foreground arena default priority
`tbb::task_arena::priority::normal` and the background arena
`tbb::task_arena::priority::low`, splitting the number of available CPU cores evenly
between both of them.
The calls that invoke the actual work on both the GTSAM and Open3D side do then get
wrapped in `task_arena::execute([&](){ /* do the work */ });` and TBB takes care of
the rest.

## Scan To Map Registration

The entire point of doing bundle adjustment in the background is to build a globally
consistend pointcloud map, which `MappingSystem` can use to register new keyframes
against and provide reasonable pose priors, thus improving overall system accuracy.

Based on the current pose and overlap of the pointclouds axis-aligned bounding-boxes
(AABBs) an internal `MapppingSystem::registrationCache_` is built-up, making sure that
only the submaps with reasonable overlap are used for registration.
The cache will be marked *dirty* when a new sumbmap enters the current keyframes AABB
or one that is currently cached leaves it.
New submaps will be merged into the global map object which is then voxelized.
However, as there is no easy way to remove submaps from the merged-and-voxelized cloud,
it has to be cleared and rebuilt from scratch when submaps leave the cache.

While most of the project uses the (internally referred to as) *"legacy"* Open3D API,
i.e. `open3d::geometry::PointCloud`, the cache and scan to map registration use the
`open3d::t` Tensor-based API, as it natively implements a coarse-to-fine ICP pipeline
which is used to provide a more accurate registration result to the final scan-to-map
registration, which is done in `MappingSystem::registerScanToMap`.
See also
[MultiScaleICP](https://www.open3d.org/docs/latest/cpp_api/namespaceopen3d_1_1t_1_1pipelines_1_1registration.html#a300caad70b099cb9f5d5ce72a8ff1ecb)
.

Finally, when the resulting pose exceeds a configurable relative fitness threshold
(see `ScanToMapRegistrationConfig`), it creates a `PriorFactor<Pose3>` that places an
additional constraint on the LiDAR-IMU pose estimate.
The registration makes use of the IMU-to-LiDAR extrinsic calibration \f$ \iTl \f$.
