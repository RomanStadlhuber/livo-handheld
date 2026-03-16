# Deriving the Jacobians {#jacobians}

This page contains the derivations of the jacobians implemented in
the Point-To-Plane residual factor.
For derivations of the IMU-Preintegration (including jacobians),
please see
[On Manifold Preintegration for Real-Time Visual-Inertial Odometry (Forster et. al, 2015)](https://arxiv.org/abs/1512.02363).

## Important Rules

There are three major rules that will be needed to understand the
derivatives that will follow.

### 1. The Chain Rule

\f[
    \frac{d}{dx} g(f(x)) = \frac{d}{df}g(f(x)) \cdot \frac{d}{dx}f(x)
\f]


Which, in the context of this work, will apply to the squared resiudal
formulation. Informally,
\f[
    \frac{d}{d \boldsymbol{p}} \frac{1}{n_j}
    (\boldsymbol{n}^{T} \cdot \boldsymbol{p} + d)^{2} =
    \frac{2}{n_j}
    (\boldsymbol{n}^{T} \cdot \boldsymbol{p} + d)
    \cdot \boldsymbol{n}^{T}
\text{ .}\f]
Where, in what will follow, all derivatives will be computed through
the position of the LiDAR scan point in the world frame (informally
\f$ \boldsymbol{p} \f$).

### 2. Linearizing the Exponential Map

Equations that involve pose-deltas will frequently need to work with
the exponential map.  In order to compute derivatiles from \f$ exp \f$,
it is approximated linearly

\f[
 X \cdot exp \{ \tau \} \simeq
 X \cdot \left[ I + \tau \right]
\text{ .}\f]

Which follows directly from the exponential series equation

\f[
exp\{\tau\} = I + \tau + \frac{\tau^{2}}{2!} + \frac{\tau^{3}}{3!} + ...
\text{ ,}\f]

where generally \f$ \tau \f$ can be an element of any Lie groups tangent
space and \f$ I \f$ is the identity mapping within that tangent space.
As GTSAM works with local increments, this work uses the right-applied
\f$ exp \f$ with elements of the local tangent space (e.g. velocities or
rates in the body frame) as opposed to just the Lie algebra.

### 3. Skew Symmetric Identity

The Lie group \f$ SO(3) \f$ represents twists (specifically, angular rates)
as skew-symmetric matrices as elements on the tangent space,
while an isometry exists that represents any skew-symmetric matrix as a
vector and vice-versa.

In 3D, skew symmetric matrices are equivalent to the cross-product, i.e.
left-applying \f$ {\bigl[\boldsymbol{a}\bigr]}_{\times} \cdot \boldsymbol{b} \f$
is the same as \f$ \boldsymbol{a} \times \boldsymbol{b} \f$.
Skew symmetric matrices have the property that
\f$ \boldsymbol{A}^{T} = -\boldsymbol{A}\f$ and from that directly follows
that

\f[
  {\bigl[\boldsymbol{a}\bigr]}_{\times} \cdot \boldsymbol{b}  =
  -  {\bigl[\boldsymbol{b}\bigr]}_{\times} \cdot \boldsymbol{a}
\text{ ,}\f]

which can be intuitively explained by the fact that flipping the operands
of the cross-product similarly yields

\f[
    \boldsymbol{a} \times \boldsymbol{b} =
    - \boldsymbol{b} \times \boldsymbol{a}

\text{ .}\f]

This property will be used to turn skew-symmetric expressions of body rates
\f$ {\bigl[\boldsymbol{\omega}\bigr]}_{\times} \cdot \boldsymbol{p} \f$ into
vectors (\f$ = - {\bigl[\boldsymbol{p}\bigr]}_{\times} \cdot \boldsymbol{\omega} \f$)
to compute their derivative with respect to \f$ \omega \f$.


## The Residual

We begin with the residual function itself, for which the all LiDAR
point-related derivatives will be computed.

The mapping system expresses features as multiple clusters, the location and
parameters of these cluster are not *estimated* explicitly, rather they are
inferred from the pose variables \f$ X(i) \f$ themselves. I.e. the cluster
center, the plane normal vector and the noise parameter \f$ \sigma_j \f$ will
be estimated given the coordinates of the points in the LiDAR frames, composed
with the current IMU body-pose in the world frame.
All residuals are calculated in the world frame, meaning the cluster center
and the plane normal (orientation) are explained in the world frame as well.

The \f$ n_j \f$ points of cluster \f$ j \f$ are associated with one keyframe in
the sliding window, each. On a high level, the residual for that cluster factor
is the mean of the squared point-to-plane distances

\f[
r_{j}(X(n), ..., X(m)) = \frac{1}{n_{j}} \sum_{k}
({\boldsymbol{n}_{j}}^{T} \cdot {}^{w}\boldsymbol{p}_{k} + d_j)^{2}
\text{ .} \f]

Where \f$ {\boldsymbol{n}_{j}}^{T} \cdot {}^{w}\boldsymbol{p}_{k} + d_j \f$
results from the cluster center \f$ {}^{w}\boldsymbol{c}_{k} \f$, meaning
\f$
{\boldsymbol{n}_{j}}^{T} \cdot {}^{w}\boldsymbol{p}_{k}
- {\boldsymbol{n}_{j}}^{T} \cdot {}^{w}\boldsymbol{c}_{k}
\f$ -- i.e.
\f$ d_{j} = - {\boldsymbol{n}_{j}}^{T} \cdot {}^{w}\boldsymbol{c}_{k} \f$,
which at inference-time is treated as a constant and will therefore be ignored
when computing the jacobian expressions.

## Pose Jacobian

The LiDAR point plane clusters influence the estimator poses
\f$ X(i) \f$, that is
\f$
{}^{w}\boldsymbol{T}_{I} =
\left[ 
    {}^{w}\boldsymbol{R}_{I} \mid {}^{w}\boldsymbol{p}_{I}
\right]
\in SE(3)
\f$ .


## Extrinsic Calibration Jacobian

This uses the same following as the pose jacobians, due to the LiDAR
pose being obtained by composing the IMU-pose with the IMU-to-LiDAR
extrinsic calibration
\f$
{}^{w}\boldsymbol{T}_{L} = {}^{w}\boldsymbol{T}_{I} \cdot {}^{I}\boldsymbol{T}_{L}
\f$ .

## Temporal Calibration Jacobian

There are different ways to find residuals for extrinsic calibration,
also note that [MSC-LIO](https://arxiv.org/abs/2407.07589) (Eqs. (21)
and (22)) does it by estimating velocities of associations they deem
to be the same physical point.

This work however builds on top of the same residual that is used for
all planar clusters. The core idea is that the LiDAR pose follows from
extrapolating the preintegrated IMU pose by a time-interval
\f$ {}^{I}t_{L} \f$, that is the temporal IMU-to-LiDAR calibration.

