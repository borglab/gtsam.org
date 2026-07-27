---
layout: gtsam-post
title:  "The Manifold Kalman Filter Hierarchy, Part 3: Equivariant Filters"
---

*Authors*: [Frank Dellaert](https://dellaert.github.io/) and [Rohan Bansal](https://rbansal.dev/).
*GTSAM Contributors*: Jennifer Oum, Darshan Rajasekaran, Alessandro Fornasier (on whose code our examples are based).

<!-- - TOC -->
{:toc}

Last week we zoomed out and talked about [STAG](/2026/04/21/factor-graphs-and-world-models.html): state, dynamics, measurements, objectives, and how factor graphs can tie those pieces together. We discussed fixed-lag smoothing, which optimizes over a short window of recent states using measurement factors. This week we zoom back in on single-state *filters*, specifically filters in which the state lives on a **manifold**.

In GTSAM, we now also provide an **equivariant filter**. The word *equivariant* has the same basic meaning here as it does in equivariant neural networks: if you transform the input, the output should transform in the corresponding way. In filtering, the payoff is not just elegance. If the dynamics and measurements respect the symmetry, the filter can express its error around a fixed reference state instead of around the current, possibly wrong, estimate.

This post is meant as a quick tutorial introduction for GTSAM users. The foundations were developed by Mahony, Hamel, and Trumpf in ["Equivariant Systems Theory and Observer Design"](https://arxiv.org/abs/2006.08276), and the main EqF reference is ["Equivariant Filter (EqF)"](https://arxiv.org/abs/2010.14666) by van Goor, Hamel, and Mahony. An earlier short paper, ["Equivariant Filter Design for Kinematic Systems on Lie Groups"](https://arxiv.org/abs/2004.00828) by Mahony and Trumpf, is also useful background.

## EqF: the Missing Piece in the Hierarchy

This is the third post in the manifold Kalman filter hierarchy series. [Part 1](/2026/03/09/manifold-kf-part1.html) introduced `ManifoldEKF`, `LieGroupEKF`, `InvariantEKF`, and `LeftLinearEKF`; [Part 2](/2026/03/17/legged-state-estimation-part2.html) showed why invariant filtering matters in legged state estimation.

The **Equivariant Filter (EqF)** is the final missing piece. The `ManifoldEKF` can handle a broad range of manifold states, including `Unit3`, but its covariance propagation is tied to the current estimate because the transition Jacobian is computed in the tangent space at the current linearization point. A bad estimate can therefore give you a bad local error model. The `InvariantEKF` and `LeftLinearEKF` get the invariant-error behavior we like, where the propagated error can be much less dependent on the current estimate, but they assume the state itself is a Lie group.

The new `EquivariantFilter` fills the gap: it is an error-state Kalman filter where the physical state can live on a general manifold $\mathcal{M}$, while a separate symmetry group $\mathcal{G}$ acts on that state. The group action gives the filter an invariant-style error without requiring the state itself to be a group.

## A Simple Example

Imagine a robot carrying a gyroscope and a magnetometer. We want to estimate the direction of the magnetic field in the robot's body frame. That direction is a unit vector

$$
\eta \in \mathbb{S}^2,
$$

with dynamics

$$
\dot{\eta} = -\Omega^\times \eta,
$$

and measurement

$$
y = c_m \eta.
$$

Here, $\Omega$ is the angular velocity measured by the gyroscope. The matrix $\Omega^\times$ is the cross-product matrix, so the dynamics say that the magnetic-field direction appears to rotate in the robot's body frame as the robot turns. The measurement equation says the magnetometer observes that same direction, scaled by a constant magnetic-field strength $c_m$.

*The important detail is that the state is not a group*. It is a point on the sphere:

$$
\mathcal{M} = \mathbb{S}^2.
$$

In GTSAM, that is a `Unit3`. You can perturb it locally with two tangent coordinates, but you cannot compose two directions and get another direction in the group-theoretic sense. So this is a natural state for `ManifoldEKF`, not for `InvariantEKF`.

However, rotations act on directions. The group

$$
\mathcal{G} = SO(3)
$$

moves points around on $\mathbb{S}^2$. In GTSAM terms, the physical state can be `Unit3`, while the symmetry group can be `Rot3`. This is exactly the kind of situation EqF is designed for: the state is not a group, but a group still acts on it.

## The EqF Idea

<figure class="center" style="width: 100%; max-width: 900px;">
  <img src="/assets/images/equivariant/group-action-manifold.png"
    alt="A symmetry group hovering above a physical manifold, with red dynamics arrows from timestep k to k plus one and purple group-action arrows from the reference state to each estimate"
    style="width: 100%;" />
  <figcaption>EqF keeps the physical estimate on the manifold $\mathcal{M}$, while the lifted estimate evolves on the symmetry group $\mathcal{G}$. The dashed purple arrows show the group actions from the reference state to the estimates at <i>k</i> and <i>k+1</i>, and the dashed gray lift arrow maps the manifold dynamics to the corresponding group dynamics.</figcaption>
</figure>
<br />

There are three geometric objects in play. The physical state lives on a manifold $\mathcal{M}$, the symmetry lives in a group $\mathcal{G}$, and the group action

$$
\phi : \mathcal{G} \times \mathcal{M} \rightarrow \mathcal{M}
$$

tells us how a group element moves a state. For the sphere example, the state is a direction $\eta \in \mathbb{S}^2$, the group is $SO(3)$, and the action rotates the direction. This is the extra structure beyond `ManifoldEKF`: the state does not have to be a group, but there must be a useful group action on it.

Instead of storing the estimate only as a state $\hat{\xi}$ on the manifold, the EqF also stores a group element $\hat{g}$ and applies it to a fixed reference state $\xi^\circ$:

$$
\hat{\xi} = \phi(\hat{g}, \xi^\circ).
$$

In the sphere example, $\xi^\circ$ might be the north pole direction, and $\hat{g}$ is a rotation that moves that reference direction to the current estimate.

In the GTSAM implementation, the physical estimate is still an element of `M`, but EqF also maintains a group element `g_`; applying `g_` to `xi_ref_` recovers the current state estimate.

This also gives a natural error:

$$
e = \phi(\hat{g}^{-1}, \xi).
$$

If the estimate is correct, the error lands back at the fixed reference point:

$$
e = \xi^\circ.
$$

That fixed reference point is the key. In a usual manifold EKF, the tangent space and linearization move with the current estimate. In an *equivariant* filter, the symmetry action lets us express the error around a fixed origin. Equivariance is the consistency condition that makes this legitimate: moving the state by the symmetry and then applying the dynamics or measurement model must match applying the model first and then moving the result in the corresponding way. When that holds, covariance propagation is less tied to the filter's current guess.

The runtime loop still looks like an EKF: predict the state, propagate covariance, compare predicted and observed measurements, compute a Kalman gain, and apply a correction. The difference is where the geometry enters. During prediction, the **lift** $\Lambda$ converts physical dynamics into a small motion in the group. In the attitude-direction example, the gyroscope moves the group estimate, and the direction estimate follows from the group action. During update, the Kalman correction is lifted back through the group instead of directly nudging arbitrary coordinates on the sphere.

## The GTSAM Template

The generic implementation lives in [`gtsam/navigation/EquivariantFilter.h`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/EquivariantFilter.h). It is templated on $\mathcal{M}$, the physical manifold state type, and `Symmetry`, the functor that defines the group action. The class inherits from `ManifoldEKF<M>`, so it reuses the same covariance, Kalman gain, and Joseph-form update machinery from the base hierarchy.

Importantly, the covariance has the dimension of `M`, not the dimension of `Symmetry::Group`: the group is not replacing the physical state with a larger Kalman state. It is used to transport the estimate and corrections in a way that respects the symmetry. The EqF-specific part is the additional symmetry bookkeeping:

- `Symmetry::Group` is the internal group $\mathcal{G}$
- `xi_ref_` is the fixed reference state $\xi^\circ$
- `g_` is the lifted group estimate
- `act_on_ref_(g_)` recovers the current manifold estimate
- `Dphi0_` maps group tangent perturbations down to manifold tangent perturbations
- `InnovationLift_` maps the EKF innovation back up to the group tangent space

So, you need to provide more structure than a plain `ManifoldEKF`: the physical state type, the `Symmetry` action, the lift from physical dynamics to group motion, the input and output actions needed for equivariance, and the usual process and measurement noise models. The payoff is that the filter is using the geometry of the problem rather than an arbitrary local coordinate choice.

The EqF is worth considering when all three of these are true:

1. Your state lives naturally on a manifold that is not necessarily a group.
2. A Lie group acts on that state in a meaningful way.
3. You care about the invariant-filter benefit: error and covariance propagation that are less dependent on the current estimate being right.

If the state is just a generic manifold with no useful symmetry, `ManifoldEKF` is the right abstraction. If the state is a Lie group and the dynamics have the invariant form, `InvariantEKF` or `LeftLinearEKF` might be simpler.

## Takeaway

The `EquivariantFilter` is a natural next step after the hierarchy in Part 1 and the invariant-filter application in Part 2. We now have a better answer for states such as directions on a sphere: keep the state on the physical manifold, but let a symmetry group push that state around. That gives GTSAM users a way to model states that are not groups *without* giving up on invariant filtering.

## Further Reading

- ["Equivariant Systems Theory and Observer Design"](https://arxiv.org/abs/2006.08276), Mahony, Hamel, and Trumpf
- ["Equivariant Filter (EqF)"](https://arxiv.org/abs/2010.14666), van Goor, Hamel, and Mahony
- ["Equivariant Filter Design for Kinematic Systems on Lie Groups"](https://arxiv.org/abs/2004.00828), Mahony and Trumpf
- ["Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration"](https://arxiv.org/abs/2209.12038), Fornasier, Ng, Brommer, Böhm, Mahony, and Weiss. This is the ABC paper that directly inspired the GTSAM ABC example.
- GTSAM EqF test: [`testEquivariantFilter.cpp`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/tests/testEquivariantFilter.cpp)
- A more complicated ABC EqF example in GTSAM: [Example](https://github.com/borglab/gtsam/blob/develop/examples/AbcEquivariantFilterExample.cpp), [Filter](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/geometry/ABCEquivariantFilter.h)

_Disclosure: AI was used to help draft this post._
