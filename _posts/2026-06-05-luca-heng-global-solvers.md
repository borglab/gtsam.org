---
layout: gtsam-post
title:  "Certifiable Optimization, Part 3: Global Solvers for 3D Vision"
---

Author: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

This third and final ICRA-week post zooms out to the broader global-solvers ecosystem.
[Part 1]({% post_url 2026-06-01-icra-for-workshop %}) focused on our chordal-sparsity work, which uses Bayes-tree structure to make SDP relaxations more scalable.
[Part 2]({% post_url 2026-06-03-certifiable-factor-graphs %}) focused on David Rosen's group's [Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg), which approaches related factor-graph problems through Burer-Monteiro-style low-rank optimization.
But these are only two efforts in a broader push toward global solvers for 3D vision and robot perception, a lot of it started by Luca Carlone & collaborators over the last 10 years. At ICRA this year we were very fortunate to have a workshop keynote by Heng Yang, his former Ph.D. student and now a professor at Harvard:

- [Scaling Semidefinite Relaxations for Robot Perception and Control](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/schedule) at the [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/).

So, in this Part 3, I want to draw some attention to Luca and Heng's seminal work in this area

## A short lineage

One way to see the field's trajectory is through rotation estimation, pose-graph optimization, and registration. Our 2015 ICRA paper, [Initialization Techniques for 3D SLAM](https://repository.gatech.edu/entities/publication/ae22ec18-65d1-4075-8c0b-55c694ac5467), was about making hard nonconvex SLAM problems behave better by separating out the rotation-estimation structure before running local pose-graph optimization. At MIT, David Rosen pioneered [SE-Sync](https://arxiv.org/abs/1612.07386), which showed how pose synchronization over `SE(d)` could be solved efficiently while certifying global optimality in the relevant noise regime. I also worked with David and Luca and others on [Shonan Rotation Averaging](https://arxiv.org/abs/2008.02737), that connected SDP relaxation and manifold optimization to large-scale rotation averaging.

Heng and Luca then pushed certifiable registration toward extreme outlier robustness in the context of point cloud registration, with [TEASER, by Heng Yang, Jingnan Shi, and Luca Carlone](https://arxiv.org/abs/2001.07715). This paper already has > 1100 citations, and Heng has been on a convex relaxation binge since then!

## Global solvers for 3D vision

A lot of the recent developments in this area, by Heng and others, were collected in the workshop paper [Global Solvers for 3D Vision: Foundations, Frontiers, and a Call to the Robotics Community](https://openreview.net/forum?id=D1bVYYUh8m). A longer-form version with Heng as a co-author is available on Arxiv here: [Advances in Global Solvers for 3D Vision](https://arxiv.org/abs/2602.14662).

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/global-solvers-2026/global-solvers-taxonomy.png"
    alt="Taxonomy of global solvers for 3D vision, including branch-and-bound, convex relaxation, graduated non-convexity, comparative analysis, and applications."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Global solvers for 3D vision span branch-and-bound, convex relaxation, and graduated non-convexity, with applications from Wahba's problem to bundle adjustment. Figure adapted from the Zhao et al. survey.</figcaption>
</figure>
<br />

If you're interested in knowing more, that survey is a very useful entry point. The authors introduce branch-and-bound (B&B), convex relaxation (CR), and graduated non-convexity (GNC) in the context of geometric estimation problems that show up constantly in robot perception, for example absolute and relative pose estimation, 3D registration, rotation and translation averaging, and even full bundle adjustment.

## Back to GTSAM

The recent [QP and QCQP support in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %}) is a preview of the modeling layer needed to lift, relax, decompose, and eventually certify factor-graph problems. We hope to bring certifiable optimization to GTSAM soon, and I hope this will allow more robotics people to join in the fun!

## Further browsing

- [Workshop paper: Global Solvers for 3D Vision](https://openreview.net/forum?id=D1bVYYUh8m)
- [ArXiv: Advances in Global Solvers for 3D Vision](https://arxiv.org/abs/2602.14662)
- [OpenReview: Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg)
- [ArXiv: SE-Sync](https://arxiv.org/abs/1612.07386)
- [ArXiv: TEASER](https://arxiv.org/abs/2001.07715)
- [ArXiv: Shonan Rotation Averaging](https://arxiv.org/abs/2008.02737)
- [Post: Certifiable Optimization at the ICRA Workshop]({% post_url 2026-06-01-icra-for-workshop %})
- [Post: Quadratic Programs and QCQPs in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %})

_Disclosure: AI was used to help draft this post and prepare the figure._
