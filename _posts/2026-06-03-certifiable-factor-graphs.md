---
layout: gtsam-post
title:  "Certifiable Optimization, Part 2: Certifiable Factor Graph Optimization"
---

Author: [David M. Rosen](https://david-m-rosen.github.io/)

<!-- - TOC -->
{:toc}

Certifiable estimation is rapidly maturing as a practical tool for robust robotic perception and state estimation, enabling both *fast* and *certifiably correct* (i.e. *verifiably globally optimal*) inference.  Frank's [companion post]({% post_url 2026-06-01-icra-for-workshop %}) highlights one way to implement certifiable estimators in GTSAM by exploiting the Bayes Tree  to perform chordal decomposition; here we describe complementary work from [Northeastern](https://neural.lab.northeastern.edu/) that exploits [Burer-Monteiro factorization](https://arxiv.org/abs/2410.00117) to implement certifiable estimation using the factor graph modeling and *local* optimization paradigm already familiar to users of GTSAM.


Our workshop paper is:

> [Certifiable Factor Graph Optimization](https://arxiv.org/abs/2603.01267), by [Zhexin (Jason) Xu](https://zhexin1904.github.io/), [Nikolas R. Sanderson](https://www.linkedin.com/in/niksand), [Hanna Jiamei Zhang](https://www.linkedin.com/in/hannajiameizhang/), and [David M. Rosen](https://david-m-rosen.github.io/). Workshop page: [OpenReview](https://openreview.net/forum?id=hAtI0KkdAg).

## Two paradigms for robotic state estimation

Factor graphs are well-established as the dominant paradigm for modeling and solving robotic state estimation tasks, primarily because they are so wonderfully easy to use. The factor graph abstraction that GTSAM is built on lets you easily model a wide range of estimation problems by composing a handful of standard variable and factor types; moreover, GTSAM can *automatically* synthesize and run fast local optimizers to perform inference directly from a factor graph model. The catch is *reliability*: because factor graph inference is typically performed using *local* optimization, it can sometimes silently converge to a badly wrong estimate, even on a well-posed problem.

More recently, *certifiable estimation* has emerged as a powerful new approach for implementing trustworthy robotic perception and state estimation systems.  The main idea behind certifiable estimators is to construct a *convex* (typically *semidefinite*) *approximation* of the target maximum likelihood estimation problem, and then solve this convex surrogate to recover a high-quality solution to the original estimation task.  This approach has three major advantages. First, because the surrogate is convex, it *can* be solved globally. Second, its minimizer often turns out to be an *exact, globally optimal solution* of the original problem. Third, these methods yield an *a posteriori certificate of optimality* whenever they succeed — that is, they can tell you whether they actually found the right answer.

However, the catch to using certifiable estimation methods is *effort*: the SDP relaxations underpinning these methods are typically very high-dimensional, and therefore require specialized, structure-exploiting optimization techniques in order to solve them efficiently.   The standard pipeline for deploying a high-dimensional certifiable estimator thus involves developing a problem-specific SDP relaxation, designing a custom-built *local* optimizer for its (low-dimensional) Burer-Monteiro factorization, and then wrapping that optimizer in the Riemannian Staircase to ensure global optimality.  Executing this process can require *weeks to months* of specialized effort, which must be repeated from scratch for every new problem.

## The key insight: Certifiable estimation inherits factor graph structure

The standard pipeline treats the SDP machinery and the factor graph model as separate worlds. Our central observation is that they aren't: the two transformations at the heart of certifiable estimation — Shor's relaxation and Burer-Monteiro factorization — *preserve the factor graph structure* of the problem that they start from.

The reason is almost embarrassingly simple. The maximum likelihood problems we care about can be written as *quadratically-constrained quadratic programs* (QCQPs), and the factor graph structure of such a QCQP is encoded directly in the data matrices that define it: factor connectivity shows up as *block sparsity* in the objective matrices, and the product structure of the feasible set shows up as *block-diagonal* constraint matrices. The key point is that the *same data matrices* that define the original QCQP *also* define its Shor relaxation, as well as *every Burer-Monteiro factorization* of that relaxation. None of this structure is lost along the way!

The consequence is that the Burer-Monteiro-factored Shor relaxation *automatically inherits* a factor graph structure from the original estimation problem. This induced factor graph has *identical connectivity* to the original; only the variables and factors themselves change. And these change in the simplest possible way: each is a (slightly) higher-dimensional, one-to-one algebraic transformation — a *lift* — of its counterpart in the original factor graph. For example, a rotation variable lifts to a Stiefel-manifold variable, a unit vector lifts to a higher-dimensional unit vector, a translation lifts to a (higher-dimensional) translation, and the factors lift accordingly. We call the resulting factor graph a *lifted* (or *certifiable*) factor graph.

<figure class="center" style="width: 100%;  text-align: left;">
  <img src="/assets/images/certifiable-factor-graphs/framework.png"
    alt="Framework diagram showing an input factor graph and QCQP lifted into a Burer-Monteiro factor graph inside a Riemannian Staircase."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>The factor graph for the Burer-Monteiro-factored Shor relaxation has the same connectivity as the original QCQP's factor graph; only the variable and factor types change, and these do so according to simple one-to-one algebraic transformations.</figcaption>
</figure>
<br />

## Certifiable estimation *is* factor graph optimization

Because the Burer-Monteiro-factored Shor relaxation *is itself* a factor graph, the local optimizations appearing inside the Riemannian Staircase can be carried out by an ordinary factor graph optimizer — exactly what GTSAM is built to do. Consequently, the Riemannian Staircase meta-algorithm for certifiable estimation collapses to a thin wrapper around standard local factor graph optimization:

```
Input: factor graph G for a QCQP-representable MLE problem
for p = d, d+1, ... do
    build lifted factor graph G_p for the rank-p BM factorization of G
    Y* <- LocalOptimization(G_p)        # ordinary factor graph optimization
    if Z = Y*(Y*)^T certifiably solves Shor's relaxation:
        return Y*                       # globally optimal!
end
```

For anyone already comfortable with factor graphs, this turns certifiable estimation from a research project into a modeling exercise. You take a factor graph model of your problem, replace each variable and factor with its lifted counterpart, and hand the result to the optimizer you already use. No problem-specific SDP derivation, no custom Riemannian solver, no hand-analysis of manifold geometry — and yet the Staircase still guarantees recovery of a globally optimal solution, with a certificate.

<figure class="center" style="width: 100%; max-width: 980px; text-align: center;">
  <img src="/assets/images/certifiable-factor-graphs/CertiFGO.png"
    alt="Astronaut meme: 'Certifiable estimation is just factor graph optimization?' / 'Always has been.'"
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>It's factor graphs all the way down!</figcaption>
</figure>
<br />

## Experiments

We implemented our certifiable factor graph optimization framework in GTSAM and evaluated it on three problem classes — pose-graph optimization, landmark SLAM, and range-aided SLAM — across a broad set of standard benchmarks. Two findings stand out:

- It **matches purpose-built certifiable estimators.** Our general-purpose factor graph-based  estimator recovers the same objective values and the same certified suboptimality bounds as the specialized, hand-engineered solvers [SE-Sync](https://journals.sagepub.com/doi/10.1177/0278364918784361), [Landmark SE-Sync](https://ieeexplore.ieee.org/document/9143200), and [CORA](https://ieeexplore.ieee.org/document/10665918) — all because it is solving the very same underlying relaxations.

- It **preserves the speed of local factor graph optimization.** When the initialization is already good, the method terminates at the first level of the Staircase after a single local solve (plus a [cheap verification step](https://ieeexplore.ieee.org/document/9940527)), behaving just like ordinary factor graph optimization. It only invokes the full Staircase machinery when global optimality cannot be certified, and even then the cost scales roughly linearly in the number of levels.

<figure class="center" style="width: 100%; max-width: 980px; text-align: left;">
  <img src="/assets/images/certifiable-factor-graphs/benchmarks.png"
    alt="Globally optimal solutions for pose graph optimization, landmark SLAM, and range-aided SLAM benchmarks."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Globally optimal or certifiably near-optimal solutions recovered on pose graph optimization, landmark SLAM, and range-aided SLAM benchmarks.</figcaption>
</figure>
<br />

In short, you get the global-optimality guarantees of a specialized certifiable estimator at essentially the cost of a standard local solve — and the development effort drops from the *weeks-to-months* of the bespoke pipeline to a *few hours* of modeling with standard GTSAM factor graphs.

## Why this matters for GTSAM

Certifiable factor graph optimization provides a path for bringing certifiable estimation into the software stack roboticists already use. Instead of choosing between a convenient local factor graph model and a separate bespoke certifiable solver, the same model can now support *both* fast local optimization *and* global optimality guarantees.

This fits naturally with GTSAM's recent [QP and QCQP support]({% post_url 2026-05-13-qp-qcqp-in-gtsam %}). QCQPs provide the algebraic bridge from factor graph estimation problems to semidefinite relaxations, while GTSAM already provides the factor graph abstraction, sparse optimization machinery, and many of the variable and factor types used in robotics. Our implementation in the paper is built in C++ using GTSAM, and the broader goal is to make certifiable estimation a reusable part of the factor graph workflow rather than a separate custom project for each new problem.

## Further browsing

- [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/)
- [Arxiv: Certifiable Factor Graph Optimization](https://arxiv.org/abs/2603.01267)
- [OpenReview: Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg)
- [The SLAM Handbook (Chapter 6)](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release/blob/main/main.pdf) 
- [Post: Certifiable Optimization at the ICRA Workshop]({% post_url 2026-06-01-icra-for-workshop %})
- [Post: Quadratic Programs and QCQPs in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %})

_Disclosure: AI was used to help draft this post and prepare the figures._
