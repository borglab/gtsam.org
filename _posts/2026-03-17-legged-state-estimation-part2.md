---
layout: gtsam-post
title:  "The Manifold Kalman Filter Hierarchy, Part 2: Legged State Estimation"
---

Authors: [Frank Dellaert](https://dellaert.github.io/), [Varun Agrawal](https://varunagrawal.github.io/)

<!-- - TOC -->
{:toc}

[Part I](/2026/03/09/manifold-kf-part1.html) introduced the new manifold and Lie-group Kalman filter hierarchy in GTSAM. Here we highlight one of the most impactful applications of the invariant filter in robotics: legged state estimation with an IMU and contact kinematics, in the spirit of Ross Hartley, who also wrote an earlier GTSAM post on [legged robot factors]({% post_url 2019-09-18-legged-robot-factors-part-I %}), and the later paper ["Contact-aided invariant extended Kalman filtering for robot state estimation"](https://arxiv.org/abs/1904.09251).

After introducing the filter itself, we will show how factor graphs can be leveraged for increasingly more sophisticated legged estimators: first a local graph update, then a fixed-lag smoother with one shared bias, and finally a fixed-lag smoother with a bias trajectory.

<figure class="center" style="width: 110%; max-width: 110%; margin-left: -5%;">
  <img src="/assets/images/legged-kf/stairs_side.gif"
    alt="Legged robot staircase estimation using IMU and contact measurements"
    style="width: 100%; max-width: none;" />
  <figcaption>A staircase sequence from the legged-estimation tutorial. The data source is the <a href="https://github.com/SangwooJung98/Co-RaL-Dataset">Co-RaL dataset</a>, and the animation is generated from the corresponding GTSAM notebook/example.</figcaption>
</figure>
<br />

## 1. A simplified contact-aided invariant invariant EKF

A cool idea, tracing back to [Michael Bloesch et al.](https://infoscience.epfl.ch/server/api/core/bitstreams/bb6c046d-6633-4c8c-8a5f-f8729667c6b6/content), is to treat legged pose estimation as a "foot-SLAM" problem: we should not just "use contacts", but augment the state with contact "landmarks". Our starting point in this post is Ross Hartley et al.'s later paper, ["Contact-aided invariant extended Kalman filtering for robot state estimation"](https://arxiv.org/abs/1904.09251), which does so in a way that preserves invariant dynamics by extending $SE_2(3)$. In GTSAM, the [`LeggedInvariantEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L139) follows that same spirit with an `ExtendedPose3` state containing rotation, position, velocity, and footholds.

Our implementation is deliberately simpler. Rather than taking joint angles as measurements, it treats feet as body-frame contact measurements in 3D. This is in contrast to full kinematic chain modeling of the legs which, as we show in our [Humanoids 2022 publication](https://arxiv.org/abs/2209.05644), leads to better forward kinematics estimates but requires more work to model.
In the provided example we replay the small staircase example, pushing "contact packets" into the filter on touchdown, or after `100 ms` of dead reckoning, whichever comes first. That logic lives in [buildContactReplayPlan](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

This gives a very practical invariant filter: light-weight, recursive, and geometry-aware, without forcing an update at a high frequency. Typically we are being pushed by the IMU, with occasional aiding by the foothold "landmark" sightings. The notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/) shows this as `LeggedInvariantEKF`, and the corresponding C++ replay variant is `invariant_ekf` in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

## 2. Replacing the update step with a factor graph solve

Because GTSAM is a factor-graph library, the next natural step is to replace the pure EKF contact correction with a small nonlinear solve over the current base state and all feet in contact.

That is exactly what [`LeggedInvariantIEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L262) does. The prediction side still uses the same invariant state representation as variant 1, but the measurement phase becomes an iterated local graph update. In other words, the estimator still behaves like a filter overall, yet each contact event is solved more like a tiny factor graph than a single linearized EKF correction.

<figure class="center" style="width: 78%; max-width: 78%;">
  <img src="/assets/images/legged-kf/legged-invariant-iekf-fragment.svg"
    alt="Factor-graph fragment for the invariant IEKF contact update"
    style="width: 100%;" />
  <figcaption>The local graph fragment used in the invariant graph-update variant: a predicted base state on the left, active footholds on the right, and one contact factor per foot currently in stance.</figcaption>
</figure>
<br />

This is a very GTSAM move. Rather than abandoning filtering altogether, variant 2 uses just enough factor-graph machinery to make the contact update richer and more nonlinear. The notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/) calls this `LeggedInvariantIEKF`, and the replay source exposes it as `invariant_graph` in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

## 3. From filtering to smoothing

GTSAM is "Georgia Tech **Smoothing** and Mapping". Once we see the contact update as a small inverse kinematics optimization problem, it is natural to then keep a short history and *smooth* instead of throwing information away immediately.

That is what variant 3 does. [`LeggedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L291) builds a fixed-lag window over contact events, creates foothold variables per contact episode, and links consecutive base states with preintegrated ["IMU factors"](https://borglab.github.io/gtsam/imufactor/). In the current implementation those motion links are `ImuFactor2` factors from [ImuFactor.h](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/ImuFactor.h). We now no longer exploit invariance, but we now use several steps of information at once.

<figure class="center" style="width: 82%; max-width: 82%;">
  <img src="/assets/images/legged-kf/legged-fixed-lag-single-bias.svg"
    alt="Fixed-lag smoother with one shared bias variable"
    style="width: 100%;" />
  <figcaption>Variant 3 keeps a short window over contact events, contact-episode footholds, and one shared IMU bias variable across the window.</figcaption>
</figure>
<br />

This is where the factor-graph viewpoint really starts to pay off. The filter variants summarize the past into one Gaussian belief at the current time. The smoother instead keeps a short recent history alive, which lets it combine delayed contact information with the accumulated IMU evidence between events. In the notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/), this appears as `LeggedFixedLagSmoother`, and in the replay driver it is `fixed_lag_single_bias` in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

## 4. Estimating bias

The last step is to stop assuming a single bias can explain the entire lag window. Variant 4 keeps the same contact-episode smoother structure, but upgrades the inertial side from one shared bias estimate to a bias trajectory over the window. In GTSAM that means moving from [`LeggedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L291) to [`LeggedCombinedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L375), and from `ImuFactor2`-style links to [`CombinedImuFactor`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/CombinedImuFactor.h).

<figure class="center" style="width: 82%; max-width: 82%;">
  <img src="/assets/images/legged-kf/legged-fixed-lag-combined-bias.svg"
    alt="Fixed-lag smoother with evolving bias states"
    style="width: 100%;" />
  <figcaption>Variant 4 keeps the same sliding-window idea, but replaces the shared bias with a bias trajectory and `CombinedImuFactor` links between consecutive events.</figcaption>
</figure>
<br />

Conceptually, the change is simple but important. Variant 3 says: "within this lag, one bias estimate is good enough." Variant 4 says: "let the bias evolve, and estimate that evolution explicitly." In the replay source this final estimator is the `fixed_lag_combined_bias` variant in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp). That makes it the most expressive of the four reference implementations, and also the closest to the direction many serious legged-estimation systems eventually need to go.

## Closing

This sequence of estimators is a nice example of why the filter hierarchy in Part I was worth building in the first place. The invariant structure gives a strong filtering baseline for contact-aided navigation, the local graph fragment makes the contact update more faithful without giving up online behavior, and the fixed-lag smoothers show how naturally the same ideas extend into fuller factor-graph inference.

By providing these reference implementations for leg robot state estimation, we hope to lower the barrier of entry to using these or implementing your own variants on top of them.

If you want to explore more, start with the notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/), then look at the C++ replay driver [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp) and the declarations of [`LeggedInvariantEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L139), [`LeggedInvariantIEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L262), [`LeggedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L291), and [`LeggedCombinedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h#L375).

_Disclosure: AI was used to help draft this post._
