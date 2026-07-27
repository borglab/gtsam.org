---
layout: gtsam-post
title:  "The Manifold Kalman Filter Hierarchy, Part 1"
---

Authors: [Frank Dellaert](https://dellaert.github.io/), Scott Baker

<!-- - TOC -->
{:toc}

GTSAM now has a small but useful hierarchy of Kalman filters for states that do not live in a plain vector space. This post is the first in a short series on that hierarchy. Here I will start with the classes that are already useful today for manifold and Lie-group filtering:

- `ManifoldEKF`
- `LieGroupEKF`
- `InvariantEKF`
- `LeftLinearEKF`

Part 2 discusses legged state estimation as an application of invariant filtering: [The Manifold Kalman Filter Hierarchy, Part 2: Legged State Estimation](/2026/03/17/legged-state-estimation-part2.html). Part 3 will cover the newer equivariant filters.

<figure class="center" style="width: 122%; max-width: 122%; margin-left: -11%;">
  <img src="/assets/images/manifold-kf/splash.png"
    alt="Kalman filtering on a manifold with predict, update, and reset steps"
    style="width: 100%; max-width: none;" />
  <figcaption>The key idea is to keep the state on the manifold while carrying the covariance in the local tangent space, then move that covariance forward through prediction, update, and reset.</figcaption>
</figure>
<br />

## Why a hierarchy?

The classical EKF assumes the state is a vector in $R^n$. That is fine for many systems, but it is a bad fit for rotations, poses, and navigation states. In those cases the state lives on a manifold, and often on a Lie group, so the filter needs to respect that geometry.

GTSAM's new hierarchy separates the common update logic from increasingly structured prediction models:

1. `ManifoldEKF` handles the generic case: state on a manifold, covariance in the tangent space.
2. `LieGroupEKF` specializes prediction to Lie groups with state-dependent dynamics.
3. `InvariantEKF` is a particularly important case where the prediction is pure group composition and the linearized error dynamics become state independent.
4. `LeftLinearEKF` generalizes that with a left-linear prediction model $X_{k+1} = W \phi(X_k) U$.

A more detailed starting point in the tree is the note [EKF-variants.md](https://borglab.github.io/gtsam/ekf-variants/). In a GTSAM context the main motivation is really IMU integration and `NavState`: once you want filtering on rotation, position, and velocity together, the plain vector-space EKF viewpoint starts to become awkward.

## 1. ManifoldEKF

[`ManifoldEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/ManifoldEKF.h) is the base class. It does not assume a vector-space state. Instead, it stores:

- a state $X$ on a manifold $M$
- a covariance $P$ in the tangent space at $X$

That is the right abstraction for objects like `Unit3`, `Pose2`, `Pose3`, `Rot3`, and `NavState`. It also works perfectly well for ordinary vector states such as `gtsam::Vector`, so this is not only about fancy geometry.

The predict step is intentionally minimal: you supply the next state and the transition Jacobian in local coordinates,

$$
X_{k+1} = X_{\text{next}}, \qquad P_{k+1} = F_k P_k F_k^T + Q_k.
$$

The figure above gives an overview of the algorithm. The <span style="color: #c62828;">red</span> arrow is the predict step on the state manifold, the <span style="color: #2e7d32;">green</span> arrow is the measurement update in the tangent space, and the <span style="color: #1565c0;">blue</span> arrow is the reset step that moves the corrected tangent-space estimate back to the manifold.

Tangent spaces are key to the working of the filter, as they provide local vector spaces in which we can use the "good old" EKF equations. In the figure, the <span style="color: #c62828;">red **prediction** moves from $T_{\mu_k}\mathcal{M}$ to the predicted state on the manifold</span>, after which the filter works in the tangent space at that predicted state. The <span style="color: #2e7d32;">green measurement **update** fuses the a measurement likelihood with the predictive density to produce a corrected estimate, *still* in the tangent space at the predicted state</span>. The actual `retract` happens only in the <span style="color: #1565c0;">blue **reset** step, where that corrected tangent-space estimate is pushed back onto the manifold and the covariance is transported to the new tangent space at the updated state</span>.

This is the core idea behind the whole hierarchy: keep the estimate on the manifold, do uncertainty calculations in the tangent space, and transport covariance correctly when the base point changes. The underlying probability model is the [Concentrated Gaussian](https://borglab.github.io/gtsam/concentratedgaussian/).

Useful places to look:

- Header: [ManifoldEKF.h](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/ManifoldEKF.h)
- Tests: [testManifoldEKF.cpp](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/tests/testManifoldEKF.cpp)

The tests are worth a look because they show the class on both `Unit3` and `Pose2`, which makes the base abstraction very concrete. While `Unit3` is a manifold (the sphere), the `Pose2` class in GTSAM corresponds to the Lie group $SE(2)$, so the next filter explicitly acknowledges that.

## 2. LieGroupEKF

[`LieGroupEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LieGroupEKF.h) is the next step up. It covers the important middle case: Lie-group states with state-dependent dynamics.

This is the right abstraction when the state lives on a Lie group, but the prediction model still depends on the current state estimate. In other words, it keeps the geometry of the state space, but does not yet assume the extra symmetry structure that makes invariant filtering possible.

Useful sources here are:

- Header: [LieGroupEKF.h](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LieGroupEKF.h)
- Tests: [testLieGroupEKF.cpp](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/tests/testLieGroupEKF.cpp)
- Example: [GEKF_Rot3Example.cpp](https://github.com/borglab/gtsam/blob/develop/examples/GEKF_Rot3Example.cpp)

That example uses a `Rot3` state with state-dependent dynamics and a magnetometer update. It shows that not every useful Lie-group filter needs to be invariant.

## 3. InvariantEKF

[`InvariantEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/InvariantEKF.h) is the most immediately useful class in the current set. It uses the left-invariant prediction structure emphasized by Barrau and Bonnabel; a good entry point is the survey-style tutorial ["The Invariant Extended Kalman Filter as a Stable Observer"](https://arxiv.org/abs/1410.1465) and the related IEKF papers by Barrau and Bonnabel.

In the simplest view, this handles cases where we can write the dynamics in terms of group composition,

$$
X_{k+1} = W_k X_k U_k.
$$

with $W_k$ and $U_k$ elements of the group themselves. In the examples below $W_k$ is due to *gravity*, acting in the navigation/world frame, and $U_k$ updates the state using IMU measurements in the body frame.

The key payoff is that the error dynamics are state independent. For the dynamics above, the linearized propagation Jacobian is

$$
F_k = \mathop{Ad}_{U_k^{-1}},
$$

so it depends only on the increment $U_k$, not on the current state estimate $X_k$.

This is why invariant filtering tends to be more robust on Lie groups: the error propagation is correct even if the filter state is wrong, which is especially important in transients, e.g., at startup.

For hands-on exploration, start with the notebooks:

- Notebook: [InvariantEKF.ipynb](https://borglab.github.io/gtsam/invariantekf/)
- Notebook: [Gal3ImuEKF.ipynb](https://borglab.github.io/gtsam/gal3imuekf/)
- Notebook: [Gal3ImuExample.ipynb](https://borglab.github.io/gtsam/gal3imuexample/)
- Notebook: [Gal3ImuASVExample.ipynb](https://borglab.github.io/gtsam/gal3imuasvexample/)

Then look at the concrete repository examples:

- [IEKF_SE2Example.cpp](https://github.com/borglab/gtsam/blob/develop/examples/IEKF_SE2Example.cpp): left-invariant filtering on `Pose2` with odometry prediction and GPS-like updates
- [IEKF_NavstateExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/IEKF_NavstateExample.cpp): invariant filtering on `NavState` with IMU prediction and GPS update

The `InvariantEKF`` is deliberately more restricted than the full theory, and focuses on the clean **left-invariant** prediction cases that fit naturally in the current GTSAM API. That is a narrower target than the IEKF paper treatment, which uses the concept of group-affine dynamics. The next filter considerably widens that scope based on a later paper.

## 4. LeftLinearEKF

[`LeftLinearEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeftLinearEKF.h) is a more general extension of the same line of thinking, introduced in the Barrau-Bonnabel paper ["Linear Observed Systems on Groups"](https://www.sciencedirect.com/science/article/pii/S0167691119300805).
In the GTSAM context, the motivation for this extra concept is `NavState`: it is exactly what lets us model the autonomous part of navigation dynamics cleanly.

In short, it is for Lie-group systems whose discrete dynamics can be written as

$$
X_{k+1} = W_k \, \phi(X_k) \, U_k,
$$

where $\phi$ is an **automorphism** of the group.
That is a big word, but for `NavState` the key idea is simple: position advances by velocity in the autonomous part of the dynamics,

$$
p_{k+1} = p_k + v_k \Delta t.
$$

Viewed this way, `InvariantEKF` is really a special case of `LeftLinearEKF`: the case where the automorphism part is trivial, so the prediction collapses to group composition. I wanted to introduce `InvariantEKF` first because it is likely to be the more useful entry point for most users, but mathematically `LeftLinearEKF` is the more general object.

For hands-on exploration, start with the notebooks:

- Notebook: [NavStateImuEKF.ipynb](https://borglab.github.io/gtsam/navstateimuekf/)
- Notebook: [NavStateImuExample.ipynb](https://borglab.github.io/gtsam/navstateimuexample/)
- Notebook: [NavStateImuASVExample.ipynb](https://borglab.github.io/gtsam/navstateimuasvexample/)

Then look at the class and tests:

- Class: [NavStateImuEKF.h](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/NavStateImuEKF.h)
- Test: [testLeftLinearEKF.cpp](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/tests/testLeftLinearEKF.cpp)

So even if you never instantiate `LeftLinearEKF` directly, it is already paying for itself as the mathematical backbone for more specialized filters.

## Part 2

This hierarchy does not stop at invariant filtering. Before moving on to equivariant filters, we discuss legged state estimation as a concrete application in [Part 2](/2026/03/17/legged-state-estimation-part2.html). The next step after that is equivariant filtering, where symmetry is used even more explicitly. GTSAM already has the machinery for that as well, including:

- [`EquivariantFilter`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/EquivariantFilter.h)
- [testEquivariantFilter.cpp](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/tests/testEquivariantFilter.cpp)
- [AbcEquivariantFilterExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/AbcEquivariantFilterExample.cpp)

We will cover that in detail in part three.

_Disclosure: AI was used to help draft this post._
