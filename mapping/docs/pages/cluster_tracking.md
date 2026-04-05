# Cluster Tracking {#cluster_tracking}

This page describes how the LiDAR frontend tracks planar feature clusters across
keyframes and how those clusters become constraints in the factor graph.

## Cluster Lifecycle

Each cluster represents a single observed plane, identified by one representative
point per keyframe. Clusters progress through a state machine:

```
Premature ──► Tracked ──► Idle ──► ShiftedIdle
    │             │          │          │
    │             ▼          ▼          ▼
    └──────────(stays)     Pruned ◄────┘
```

| State | Meaning | Factor action |
|-------|---------|---------------|
| Premature | Fewer than `min_points` observations | None |
| Tracked | Successfully associated with the latest keyframe | Create or replace factor |
| Idle | Not tracked in the latest keyframe, but still valid | Update plane parameters in-place |
| ShiftedIdle | Idle, but a keyframe was marginalized from it | Replace factor with reduced key set |
| Pruned | Fewer than 3 points remain, or validation failed | Remove factor |


## Cluster Creation

New clusters are seeded from a **lagged** keyframe (default: 2 keyframes behind the
current one). This delay gives the points time to mature through multiple optimizer
passes before they contribute to the cluster parameters and associated residuals.


## KNN Tracking Pipeline

When a new keyframe is created, `LidarFrontend::trackScanPointsToClusters` runs
the following for each non-pruned cluster:

1. **Query point selection** -- Use the **oldest** point in the cluster (most stable
   association) as the KNN query.

2. **KNN search** -- Find the `knn_neighbors` (default 5) nearest points in the
   new keyframe's submap.

3. **Local plane fit** -- Call `planeFitSVD` on the KNN neighborhood:
   \f[
     \underset{\bvec{n}}{\text{min}} \;\sum_{i} \bigl(\bvec{n}^\top (\bvec{p}_i - \bvec{c})\bigr)^2
     \quad\Longrightarrow\quad
     \bvec{n} = \bvec{v}_3 \text{ (smallest right-singular vector of centered points)}
   \f]
   Reject if the fit is invalid or `planeTrackThickness > max_plane_thickness`.

4. **Outlier test** -- For non-premature clusters, check the second-nearest
   neighbor against the existing cluster plane:
   \f[
     \left| \bvec{n}_{\text{cluster}}^\top (\bvec{p}_{\text{KNN}} - \bvec{c}_{\text{cluster}}) \right|
     \;<\; 3\,\sigma_{\text{cluster}}
   \f]

5. **Provisional point addition** -- Add the second-nearest neighbor (not the
   closest, for noise robustness) to the cluster and record the local plane
   thickness in the history.

6. **Global plane fit** -- Once the cluster has \f$ \geq \f$ `min_points`
   observations, refit a plane through **all** cluster points across all
   keyframes.

7. **Normal consistency check** -- Accept only if:
   \f[
     \left| \bvec{n}_{\text{new}} \cdot \bvec{n}_{\text{existing}} \right|
     \;>\; \texttt{thresh_normal}
   \f]
   Otherwise undo the provisional addition and set state to `Idle`.


## SVD Plane Fitting

During the tracking process itself, the plane parameters of adjacent points are
computed and validated before they are used.
This prevents creating clusters with noisy, inconsistend plane normals and centers,
which could degrade performance.

`planeFitSVD` computes a least-squares plane from a set of 3D points:

1. Compute centroid \f$ \bvec{c} \f$.
2. Build the centered matrix \f$ \mtx{M} \in \mathbb{R}^{N \times 3} \f$.
3. Thin SVD: \f$ \mtx{M} = \mtx{U} \mtx{\Sigma} \mtx{V}^\top \f$.
4. The plane normal is \f$ \bvec{v}_3 \f$ (column of \f$ \mtx{V} \f$
   corresponding to the smallest singular value \f$ \sigma_3 \f$).

Validity checks:
- **Planarity:** \f$ \sigma_3 / \sigma_2 \leq 0.1 \f$ (flat, not a blob)
- **Non-linearity:** \f$ \sigma_2 / \sigma_1 \geq 0.5 \f$ (2D spread, not a line)

Plane thickness (used as a noise proxy):
\f[
  \Gamma_{\text{fit}} = \frac{1}{N} \sum_{i=1}^{N} \bigl(\bvec{n}^\top (\bvec{p}_i - \bvec{c})\bigr)^2
\f]


## Adaptive Noise Model

The noise sigma for each cluster's `PointToPlaneFactor` is derived from the
plane thickness measurements accumulated during tracking.

### Per-track thickness measurement

Each time a cluster is successfully tracked in a new keyframe (step 5 of the
KNN pipeline), a **local plane thickness** \f$ t_k \f$ is computed from the KNN
plane fit on the adjacent points in that keyframe's submap:

\f[
  \Gamma_k = \frac{1}{N_{\text{KNN}}} \sum_{i=1}^{N_{\text{KNN}}}
    \bigl(\bvec{n}_{\text{KNN}}^\top (\bvec{p}_i - \bvec{c}_{\text{KNN}})\bigr)^2
\f]

This value captures how "thick" (noisy) the local surface patch is at the point
where the cluster was tracked. It is appended to the cluster's
`clusterPlaneThicknessHistory_`, building up a per-cluster time series of
thickness observations.

### Cluster thickness from history

The overall thickness of the \f$ j \f$ *-th* cluster is the mean of the squared historical measurements:

\f[
  \Gamma_{j} = \frac{1}{K} \sum_{k=1}^{K} \Gamma_k^2
\f]

where \f$ K \f$ is the number of entries in the thickness history. This
averaging smooths out frame-to-frame noise while preserving the trend: a
cluster on a rough surface accumulates consistently large \f$ t_k \f$ values,
while a cluster on a clean wall stays small.

When a keyframe is marginalized, the oldest history entry is removed
(`pop_front`), so the thickness estimate always reflects only the active
sliding window.

### Adaptive sigma

The cluster sigma used in the `PointToPlaneFactor` noise model is:

\f[
  \sigma_{\text{cluster}} = \bigl(0.5 \cdot \Gamma_j\,\bigr)^{1/4}
\f]

which is based on Eq. *(19)* from the [MSC-LIO](https://arxiv.org/abs/2407.07589) paper.

The resulting \f$ \sigma_{\text{cluster}} \f$ is wrapped in an isotropic noise
model with a Geman-McClure robust kernel (\f$ c = 1.0 \f$), which further
down-weights large residuals from occasional outlier tracks.
