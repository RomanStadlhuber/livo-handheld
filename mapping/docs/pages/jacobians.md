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
    \frac{d}{d \bvec{p}} \frac{1}{n_j}
    (\bvec{n}^{T} \cdot \bvec{p} + d)^{2} =
    \frac{2}{n_j}
    (\bvec{n}^{T} \cdot \bvec{p} + d)
    \cdot \bvec{n}^{T}
\text{ .}\f]
Where, in what will follow, all derivatives will be computed through
the position of the LiDAR scan point in the world frame (informally
\f$ \bvec{p} \f$).

### 2. Linearizing the Exponential Map

Equations that involve pose-deltas will frequently need to work with
the exponential map.  In order to compute derivatiles from \f$ exp \f$,
it is approximated linearly

\f[
 X \cdot \exp \{ \tau \} \approx
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
left-applying \f$ \skewOf{\bvec{a}} \cdot \bvec{b} \f$
is the same as \f$ \bvec{a} \times \bvec{b} \f$.
Skew symmetric matrices have the property that
\f$ \mtx{A}^{T} = -\mtx{A}\f$ and from that directly follows
that

\f[
  \skewOf{\bvec{a}} \cdot \bvec{b}  =
  -  \skewOf{\bvec{b}} \cdot \bvec{a}
\text{ ,}\f]

which can be intuitively explained by the fact that flipping the operands
of the cross-product similarly yields

\f[
    \bvec{a} \times \bvec{b} =
    - \bvec{b} \times \bvec{a}

\text{ .}\f]

This property will be used to turn skew-symmetric expressions of body rates
\f$ \skewOf{\omg} \cdot \bvec{p} \f$ into
vectors (\f$ = - \skewOf{\bvec{p}} \cdot \omg \f$)
to compute their derivative with respect to \f$ \omg \f$.


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
the sliding window, each. Each LiDAR point \f$ {}^{L}\bvec{p}_k \f$ is
transformed to the world frame via the IMU pose \f$ \wTi_k \f$ and the
IMU-to-LiDAR extrinsic \f$ \iTl \f$:

\f[
  {}^{w}\bvec{p}_k = \wTi_k \cdot \iTl \cdot {}^{L}\bvec{p}_k
  = \wRi_k \underbrace{\bigl(\iRl \, {}^{L}\bvec{p}_k + \ipl\bigr)}_{{}^{I}\bvec{p}_k} + {}^{w}\bvec{t}_{I,k}
\text{ ,}\f]

where \f$ {}^{I}\bvec{p}_k = \iRl \, {}^{L}\bvec{p}_k + \ipl \f$ is the point
expressed in the IMU body frame.

On a high level, the residual for that cluster factor is the mean of the
squared point-to-plane distances

\f[
r_{j}(X(n), ..., X(m)) = \frac{1}{n_{j}} \sum_{k}
({\bvec{n}_{j}}^{T} \cdot {}^{w}\bvec{p}_{k} + d_j)^{2}
\text{ .} \f]

Where \f$ {\bvec{n}_{j}}^{T} \cdot {}^{w}\bvec{p}_{k} + d_j \f$
results from the cluster center \f$ {}^{w}\bvec{c}_{k} \f$, meaning
\f$
{\bvec{n}_{j}}^{T} \cdot {}^{w}\bvec{p}_{k}
- {\bvec{n}_{j}}^{T} \cdot {}^{w}\bvec{c}_{k}
\f$ -- i.e.
\f$ d_{j} = - {\bvec{n}_{j}}^{T} \cdot {}^{w}\bvec{c}_{k} \f$,
which at inference-time is treated as a constant and will therefore be ignored
when computing the jacobian expressions.

## Pose Jacobian

The LiDAR point plane clusters influence the estimator poses
\f$ X(k) \f$, that is
\f$
\wTi_k =
\left[
    \wRi_k \mid {}^{w}\bvec{t}_{I,k}
\right]
\in SE(3)
\f$.

We perturb \f$ \wTi_k \f$ in the right tangent space with
\f$ \xi = [\delta\boldsymbol{\phi};\, \delta\bvec{t}] \in se(3) \f$:

\f[
  \wTi_k(\xi) = \wTi_k \cdot \exp \{\xi \}
\text{ .}\f]

First-order expansion of the world point (using rule 2):

\f[
  {}^{w}\bvec{p}_k(\xi) \approx
    \wRi_k \bigl(\mtx{I} + \skewOf{\delta\boldsymbol{\phi}}\bigr) {}^{I}\bvec{p}_k
    + {}^{w}\bvec{t}_{I,k}
    + \wRi_k \, \delta\bvec{t}
\text{ .}\f]

Applying the skew-symmetric identity (rule 3),
\f$ \skewOf{\delta\boldsymbol{\phi}} \cdot {}^{I}\bvec{p}_k = -\skewOf{{}^{I}\bvec{p}_k} \cdot \delta\boldsymbol{\phi} \f$,
yields the partial derivatives

\f[
  \frac{\partial\, {}^{w}\bvec{p}_k}{\partial\, \delta\boldsymbol{\phi}}
    = -\wRi_k \, \skewOf{{}^{I}\bvec{p}_k},
  \qquad
  \frac{\partial\, {}^{w}\bvec{p}_k}{\partial\, \delta\bvec{t}}
    = \wRi_k
\text{ .}\f]

The per-point signed residual is
\f$ \tilde{r}_k = \bvec{n}^{T} ({}^{w}\bvec{p}_k - \bvec{c}) \f$,
so the gradient of its square with respect to \f$ \xi_k \f$ is

\f[
  \mtx{H} =
    \frac{2}{n_j} \tilde{r}_k \left[
      -\bvec{n}^{T} \cdot \wRi_k \, \skewOf{{}^{I}\bvec{p}_k}
      \;\Big|\;
      \bvec{n}^{T} \cdot \wRi_k
    \right]
    \in \mathbb{R}^{1 \times 6}
\text{ ,}\f]

where \f$ {}^{I}\bvec{p}_k = \iRl \, {}^{L}\bvec{p}_k + \ipl \f$ and \f$ n_j \f$
is the number of points in the cluster.


## Extrinsic Calibration Jacobian

The world-frame LiDAR pose is the composition
\f$ \wTl_k = \wTi_k \cdot \iTl \f$,
so any perturbation of \f$ \iTl \f$ shifts every world point.

We perturb \f$ \iTl \f$ in the right tangent space with
\f$ \xi_{\text{ext}} = [\delta\boldsymbol{\phi};\, \delta\boldsymbol{\rho}] \in se(3) \f$:

\f[
  \iTl(\xi_{\text{ext}}) = \iTl \cdot \exp \{\xi_{\text{ext}}\}
\text{ .}\f]

First-order expansion of the IMU-frame point
\f$ {}^{I}\bvec{p}_k(\xi_{\text{ext}}) \f$:

\f[
  {}^{I}\bvec{p}_k(\xi_{\text{ext}}) \approx
    \iRl \bigl(\mtx{I} + \skewOf{\delta\boldsymbol{\phi}}\bigr) {}^{L}\bvec{p}_k
    + \ipl + \delta\boldsymbol{\rho}
\text{ .}\f]

The partial derivatives with respect to the two blocks of \f$ \xi_{\text{ext}} \f$ are

\f[
  \frac{\partial\, {}^{I}\bvec{p}_k}{\partial\, \delta\boldsymbol{\phi}}
    = \iRl \, \skewOf{{}^{L}\bvec{p}_k},
  \qquad
  \frac{\partial\, {}^{I}\bvec{p}_k}{\partial\, \delta\boldsymbol{\rho}}
    = \mtx{I}_3
\text{ ,}\f]

which, propagated to the world frame via \f$ \wRi_k \f$, give

\f[
  \frac{\partial\, {}^{w}\bvec{p}_k}{\partial\, \delta\boldsymbol{\phi}}
    = \wRi_k \iRl \, \skewOf{{}^{L}\bvec{p}_k}
    = \wRl_k \, \skewOf{{}^{L}\bvec{p}_k},
  \qquad
  \frac{\partial\, {}^{w}\bvec{p}_k}{\partial\, \delta\boldsymbol{\rho}}
    = \wRl_k
\text{ ,}\f]

where \f$ \wRl_k = \wRi_k \iRl \f$ is the world-frame rotation of the LiDAR.

The extrinsic Jacobian accumulates contributions from all \f$ n_j \f$ cluster points:

\f[
  \mtx{H}_{\text{ext}} =
    \frac{2}{n_j} \sum_{k} \tilde{r}_k \left[
      \bvec{n}^{T} \cdot \wRl_k \, \skewOf{{}^{L}\bvec{p}_k}
      \;\Big|\;
      \bvec{n}^{T} \cdot \wRl_k
    \right]
    \in \mathbb{R}^{1 \times 6}
\text{ .}\f]

## Temporal Calibration Jacobian

There are different ways to find residuals for temporal calibration,
also note that [MSC-LIO](https://arxiv.org/abs/2407.07589) (Eqs. (21)
and (22)) does it by estimating velocities of associations they deem
to be the same physical point.

This work however builds on top of the same residual that is used for
all planar clusters. The core idea is that the LiDAR pose follows from
extrapolating the preintegrated IMU pose by a time-interval
\f$ {}^{I}t_{L} \f$, that is the temporal IMU-to-LiDAR calibration.

In general, temporal calibration factors into the residual of a SLAM
system by applying feature velocities over calibration offset time deltas
to each individual feature point (3D LiDAR points which in this case are
expressed in the world frame). See Eq. (4) of
[Online Temporal Calibration for Monocular Visual-Inertial Systems](https://arxiv.org/abs/1808.00692)
for the example of a monocular VI system.

### Derivation

For each keyframe \f$ k \f$, a body-frame twist
\f$ \mtx{\tau} = (\omg_k,\, \bvec{v}_k) \f$ is precomputed from the IMU propagation.
The temporally-extrapolated IMU pose at offset \f$ dt \f$ is

\f[
  \wTi_{k,\text{extr}}(dt)
    = \wTi_k \cdot \exp \{ \mtx{\tau} \cdot dt \}
\text{ .}\f]

First-order expansion (rule 2):


\f[
    \wTi_{k,\text{extr}}(dt)
    \approx \wTi_k \cdot \left[ \mtx{I} + \mtx{\tau} \cdot dt \right]
    \text{ ,}
\f]

where the derivative computes only w.r.t. the part that multiplies with \f$ \mtx{\tau} \cdot dt \f$.
Expainding in matrix form, the velocity of point \f$  {}^{L}\bvec{p}_{k} \f$ is then expressed in
homogeneous coordinate form as

\f[
\begin{aligned}
    {}^{L}\bvec{v}_{k} &= \wTi_{k} \cdot \mtx{\tau} \cdot \iTl \cdot
    \begin{bmatrix}
    {}^{L}\bvec{p}_{k} \\ 1
    \end{bmatrix} \\[6pt]
    &= \begin{bmatrix}
        \wRi_{k} & \wpi_{k} \\
        0 & 1
    \end{bmatrix}
    \cdot
    \begin{bmatrix}
        \skewOf{\omg_{k}} &  \bvec{v}_{k} \\
        0 & 0
    \end{bmatrix}
    \cdot
    \begin{bmatrix}
        \iRl & \ipl \\
        0 & 1
    \end{bmatrix}
    \cdot
    \begin{bmatrix}
        {}^{L}\bvec{p}_{k} \\ 1
    \end{bmatrix}
    \text{ .}
\end{aligned}
\f]



Defining the world-frame angular rate matrix \f$ \Ohm_k = \wRi_k \, \skewOf{\omg_k} \f$
as a shorthand yields

\f[
{}^{L}\bvec{v}_{k}=
\begin{bmatrix}
        \Ohm_k \iRl & \Ohm_k \ipl + \wRi_{k} \bvec{v}_{k} \\
        0 & 0
\end{bmatrix}
\cdot
\begin{bmatrix}
    {}^{L}\bvec{p}_{k} \\ 1
\end{bmatrix}
\f]

which transports the body angular rate into the world frame.
The velocity of the world point \f$ {}^{w}\bvec{p}_k \f$
with respect to \f$ dt \f$ is then

\f[
  \boldsymbol{\phi}_k
    = \frac{\partial\, {}^{w}\bvec{p}_k}{\partial\, dt}
    = \Ohm_k \, {}^{I}\bvec{p}_k + \wRi_k \, \bvec{v}_k
    = \Ohm_k \bigl(\iRl \, {}^{L}\bvec{p}_k + \ipl\bigr) + \wRi_k \, \bvec{v}_k
\text{ .}\f]

The temporal Jacobian is a scalar that accumulates over all \f$ n_j \f$ cluster points:

\f[
  \mtx{H}_{dt}
    = \frac{2}{n_j} \sum_{k} \tilde{r}_k \cdot \bvec{n}^{T} \boldsymbol{\phi}_k
    \in \mathbb{R}^{1 \times 1}
\text{ .}\f]

