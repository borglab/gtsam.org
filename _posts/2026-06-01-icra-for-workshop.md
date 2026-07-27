---
layout: gtsam-post
title:  "Certifiable Optimization, Part 1: Exploiting Chordal Sparsity with Bayes Trees"
---

Author: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

This post focuses on our chordal-sparsity paper, while last week's post already covered [CMC-Opt and our new legged-robot estimators]({% post_url 2026-05-26-two-new-arxiv-papers %}), including Yetong Zhang's workshop paper on constraint manifolds with corners.
Both papers will be presented at the [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/) at ICRA shows how much momentum there is right now around certifiable optimization, convex relaxation, and structure-exploiting solvers for robotics.

## Chordal sparsity

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/icra-for-2026/chordal-framework.png"
    alt="Factor graph lifted to a QCQP, converted to a Bayes tree, and decomposed into clique-wise semidefinite variables."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>From a factor graph to a lifted QCQP, then through the Bayes tree to a chordally decomposed SDP relaxation.</figcaption>
</figure>
<br />

The chordal-sparsity paper is here:

> [Exploiting Chordal Sparsity for Globally Optimal Estimation with Factor Graphs](https://arxiv.org/abs/2605.30617), by Avinash Subramanian, [Connor Holmes](https://www.linkedin.com/in/connor-holmes-538726109/), [Timothy D. Barfoot](https://asrl.utias.utoronto.ca/~tdb/), [Frank Dellaert](https://dellaert.github.io/), and [Frederike Dümbgen](https://duembgen.github.io/).

The paper combines factor-graphs and structured optimization via the Bayes-tree with the certifiable-estimation viewpoint that Connor, Tim, and Frederike have helped push forward at the University of Toronto.
Local solvers such as Gauss-Newton and Levenberg-Marquardt are fast and exploit sparsity beautifully, but they do not promise that the answer is globally optimal. Convex SDP relaxations can give global solutions or certificates, but the naive monolithic SDP is usually too expensive. 

Our contribution is to make the relaxation respect the graph structure. Starting from a GTSAM factor graph, we lift the problem to a QCQP, construct the Bayes tree through variable elimination, and use the resulting cliques to build a chordally decomposed SDP. In plain terms: instead of solving one *enormous* positive semidefinite matrix problem, we solve many smaller clique-wise matrix problems that follow the same sparse structure GTSAM already understands.

<figure class="center" style="width: 100%; max-width: 820px; text-align: center;">
  <img src="/assets/images/icra-for-2026/chordal-ring-solver-time.png"
    alt="Solver time scaling for a 3D pose-graph SLAM ring factor graph comparing monolithic SDP, chordal SDP, and local solvers."
    style="width: 90%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>For the 3D ring pose-graph example, the chordal estimator scales much better than the monolithic SDP while retaining global-optimality guarantees when the relaxation is tight.</figcaption>
</figure>
<br />

The graph above shows the benefit of using the cliques of the Bayes tree: the chordal approach is not as fast a a local solver, but it is *globally optimal*, at a cost that is vastly less than a monolithic SDP solver, which OOMs on problems of moderate size.

## The certifiable wave

The workshop has several papers that combine certifiable optimization, convex relaxations, and structure-exploiting solvers, which is fast becoming a theme in both estimation and control. Our chordal-sparsity paper is part of the "certifiable wave", but here are many other papers touching on the theme:

- [Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg), by Zhexin Xu, Nikolas R. Sanderson, Hanna Jiamei Zhang, and David M. Rosen.
- [Low-Degree Implied Equalities for Strengthening Semidefinite Relaxations](https://openreview.net/forum?id=SYsWOHYCx0), by Alexandre Amice, Bernhard Paus Graesdal, Russ Tedrake, and Pablo A. Parrilo.
- [A Generalized Theorem of the Alternative for Certifiable Optimization with Redundant Constraints](https://openreview.net/forum?id=Kp1BbGAbCG), by Hanna Jiamei Zhang, Alan Papalia, Michael Everett, and David M. Rosen.
- [Tightening Mixed-Integer Convex Relaxations for Efficient Temporal Logic Motion Planning via Logic Network Flow](https://openreview.net/forum?id=GO890Hiwc2), by Xuan Lin, Jiming Ren, Yandong Luo, Weijun Xie, and Ye Zhao.
- [Global Solvers for 3D Vision: Foundations, Frontiers, and a Call to the Robotics Community](https://openreview.net/forum?id=D1bVYYUh8m), by Zhenjun Zhao and Javier Civera.

## GTSAM preview

I am exited to announce that soon GTSAM will support certifiable estimation in a big way, based on the recent [QP and QCQP support in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %}). QCQPs are a natural bridge between factor-graph models and many convex-relaxation pipelines: once a nonconvex estimation problem is lifted into a quadratically constrained quadratic form, semidefinite relaxations and certificates can be handled ina  systematic way. Keep watching this space !

## Further browsing

- [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/)
- [Accepted contributions](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/accepted-contributions)
- [Arxiv: Exploiting Chordal Sparsity for Globally Optimal Estimation with Factor Graphs](https://arxiv.org/abs/2605.30617)
- [Post: Last week's post on CMC-Opt and legged estimators]({% post_url 2026-05-26-two-new-arxiv-papers %})
- [Post: Quadratic Programs and QCQPs in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %})

_Disclosure: AI was used to help draft this post and prepare the figures._
