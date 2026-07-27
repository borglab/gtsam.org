---
layout: gtsam-post
title:  "Speaking of Legged Robots"
---

Author: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

On June 1, I will be presenting [CMC-Opt](https://arxiv.org/abs/2605.20796), new work by [Yetong Zhang](https://www.cc.gatech.edu/people/yetong-zhang) on constraint manifolds with corners, at the [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/) at ICRA. The paper is now available on arXiv:

> [CMC-Opt: Constraint Manifold with Corners for Inequality-Constrained Optimization](https://arxiv.org/abs/2605.20796), by [Yetong Zhang](https://www.cc.gatech.edu/people/yetong-zhang) and [Frank Dellaert](https://dellaert.github.io/).

The paper presents a new way to use manifold optimization to solve large inequality-constrained optimization problems, such as state estimation and motion planning in legged systems: think quadrupeds and humanoids.

Speaking of legged systems, I'm also excited to announce more details and extensive experimental results on the four simple proprioceptive estimators for legged robots that we recently added to GTSAM. This work was done in collaboration with my recently graduated Ph.D. student [Varun Agrawal](https://varunagrawal.github.io/), and with the awesome [Chiyun Noh](https://chiyunnoh.github.io/) and [Ayoung Kim](https://ayoungk.github.io/) from the [RPM Robotics Lab](https://rpm.snu.ac.kr/) at Seoul National University (SNU):

> [Four Simple Proprioceptive Estimators for Legged Robots](https://arxiv.org/abs/2605.23100) by [Frank Dellaert](https://dellaert.github.io/), [Chiyun Noh](https://chiyunnoh.github.io/), [Varun Agrawal](https://varunagrawal.github.io/), and [Ayoung Kim](https://ayoungk.github.io/).

What these two papers have in common is that we turn the many constraints in legged systems (and robotic systems in general) into factor graphs that can be *optimized* to do both perception and planning. Some might say it's [sense-think-act with factor graphs]({% post_url 2026-04-21-factor-graphs-and-world-models %}).

## CMC-Opt at ICRA

<figure class="center" style="width: 100%; max-width: 920px; text-align: center;">
  <img src="/assets/images/arxiv-may-2026/cmc-manifold-with-corners.png"
    alt="Constraint manifolds with corners, tangent spaces, and retraction examples."
    style="width: 90%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Constraint manifolds with corners: feasible geometry, tangent spaces, and retractions.</figcaption>
</figure>
<br />

The core idea in [CMC-Opt](https://arxiv.org/abs/2605.20796) is to transform the hard problem of constrained optimization into an *unconstrained* problem directly on the feasible state space. [Yetong Zhang](https://www.cc.gatech.edu/people/yetong-zhang), who is now on the motion planning team at [Waymo](https://waymo.com/), did a Ph.D. thesis with me on using geometry to make constrained robotic inference and planning problems easier to solve.
The figure above illustrates **constraint manifolds with corners**, which can capture nonlinear equalities *and* inequalities.

<figure class="center" style="width: 100%; max-width: 920px; text-align: center;">
  <img src="/assets/images/arxiv-may-2026/cmc-quadruped-trajectory.png"
    alt="Quadruped jumping trajectory comparison for penalty optimization and CMC-Opt."
    style="width: 80%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Quadruped jumping example: penalty baseline versus CMC-Opt.</figcaption>
</figure>

Instead of using a monolithic classical constraint solver, his idea was to use the sparse graph structure and manifold optimization (as GTSAM provides) to create new manifolds out of the original variables and the constraints that involve them. Yetong created an algorithm to automatically identify "constraint-connected components" and turn them into new manifold types: lower-dimensional feasible spaces that reflect the robot’s inherent topological structure. The result is a new, coarser factor graph, which can really pay off in kinodynamic motion planning. For example, in the quadruped jumping example illustrated above, the search space drops from 32,194 dimensions to 2,260, and constraints will be satisfied *by construction*.

This is a continuation of the CM-Opt work that previously appeared at ICRA 2023 as [Constraint Manifolds for Robotic Inference and Planning](https://ieeexplore.ieee.org/document/10161024).

## Legged robot estimators

The [second paper](https://arxiv.org/abs/2605.23100) is just as exciting! In a previous post, [Legged State Estimation]({% post_url 2026-03-17-legged-state-estimation-part2 %}), I wrote about four simple *proprioceptive* estimators for legged robots, i.e., they only use an IMU and internal joint angles over time to determine the trajectory of the robot. They are basically KISS-style versions of the ideas first explored by [Michael Bloesch et al.](https://infoscience.epfl.ch/server/api/core/bitstreams/bb6c046d-6633-4c8c-8a5f-f8729667c6b6/content) and [Ross Hartley et al.](https://arxiv.org/abs/1904.09251), and our own [Humanoids 2022 publication](https://arxiv.org/abs/2209.05644), where footholds are treated as landmarks: [foot SLAM](https://en.wikipedia.org/wiki/Footloose), so to speak.

[Varun Agrawal](https://varunagrawal.github.io/) was instrumental in creating legged-robot locomotion and estimation capabilities in the same factor-graph style used elsewhere in GTSAM, and those four estimators were directly inspired by his work. All four variants [are available in GTSAM](https://borglab.github.io/gtsam/leggedestimator/).
But the question remained: how well do they perform on real, hardcore data?

<figure class="center" style="width: 100%; max-width: 720px; text-align: center;">
  <img src="/assets/images/arxiv-may-2026/garliLeo-spot-platform.png"
    alt="Boston Dynamics Spot robot used for the GaRLILEO dataset."
    style="width: 50%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>The Boston Dynamics Spot at SNU, platform used in the GaRLILEO dataset.</figcaption>
</figure>

Enter the collaboration with [Chiyun Noh](https://chiyunnoh.github.io/) and [Ayoung Kim](https://ayoungk.github.io/) at Seoul National University (SNU): their group has done amazing work in LiDAR- and radar-based state estimation and was kind enough to help us properly evaluate our new estimators, using the data they painstakingly collected with a Boston Dynamics Spot robot to create the [GaRLILEO dataset](https://garlileo.github.io/GaRLILEO/).
And, to top it off, Chiyun created a [ROS2-compatible implementation](https://github.com/ChiyunNoh/GTSAM-Legged-Estimator-ROS2), so you can try it yourself!

<figure class="center" style="width: 100%; max-width: 920px; text-align: center;">
  <img src="/assets/images/arxiv-may-2026/legged-trajectory-comparison.png"
    alt="Representative trajectory comparisons for four proprioceptive legged estimators."
    style="width: 80%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Representative trajectory comparisons for invariant filtering, local graph updates, and fixed-lag smoothing, with and without evolving bias. (a) "CorriLoop" emphasizes horizontal loop consistency, while (b) "Downstair" highlights vertical tracking during sustained elevation change.</figcaption>
</figure>

The main discovery, hinted at in the figure above: the simple GTSAM legged estimators are not half bad, one might even say they are pretty good! *In this dataset* they beat out all of the state-of-the-art estimators that we tested them against. This is absolutely not the end of the story: we are working to test these estimators in several other environments and against other recent estimators. But for something that originated as an example in my [advanced robotics class](https://dellaert.github.io/26S-AMR/), they are at the very least very good baselines to test against.

Of course, these "blind" estimators are just the basis for a practical robot perception system. You need to fuse these with an external sensor, such as vision, LiDAR, or radar, which is exactly what [Ayoung Kim](https://ayoungk.github.io/)'s group excels at doing. But it's good to start from an estimator that at least keeps very good track of where you walk while closing your eyes!

## Further browsing

- [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/)
- arXiv link: [CMC-Opt: Constraint Manifold with Corners for Inequality-Constrained Optimization](https://arxiv.org/abs/2605.20796).
- blog post: [Legged State Estimation]({% post_url 2026-03-17-legged-state-estimation-part2 %})
- arXiv link: [Four Simple Proprioceptive Estimators for Legged Robots](https://arxiv.org/abs/2605.23100).
- STAG: [sense-think-act with factor graphs]({% post_url 2026-04-21-factor-graphs-and-world-models %})

_Disclosure: AI was used to help draft this post and prepare the figures._
