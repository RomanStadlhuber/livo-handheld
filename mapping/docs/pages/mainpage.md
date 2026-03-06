# LIVO Handheld Mapping {#mainpage}

LiDAR-Inertial Visual Odometry system for real-time handheld 3D mapping.

## Overview

This system fuses data from an IMU and a LiDAR sensor to estimate 6-DoF poses
in real time. It uses a sliding-window factor graph optimized with GTSAM's
incremental fixed-lag smoother (iSAM2).

The key formula for point-to-plane residuals is:

\f$ r_{j}(x_{i}) = \mathbf{n}^\top \bigl( {}^{W}T_{I} \cdot {}^{I}T_{L} \circ {}^{L}\mathbf{p}_i - \mathbf{c} \bigr) \f$

Where \f$r_{j}\f$ is the residual of the \f$ j \f$ _-th_ cluster with respect to the \f$ i \f$ *-th* keyframe pose \f$ x_{i} \f$.

## Architecture

| Module | Description |
|--------|-------------|
| @ref frontend | IMU preintegration and LiDAR scan processing |
| @ref backend | Factor graph optimization and feature management |
| @ref factors | Custom GTSAM factors (point-to-plane) |
| @ref core | Shared types, configuration, buffers, and state |

## Detailed Documentation

- @subpage cluster_tracking -- Planar feature tracking across keyframes
- @subpage backend_isam2 -- iSAM2 configuration, update cycle, and marginalization
- @subpage recovery -- Full state recovery on tracking loss.
- @subpage core_structure -- Buffers, state management, and data flow

## Building the Documentation

```bash
cmake --build <build-dir> --target docs
```

The generated HTML is written to `docs/output/html/`.
