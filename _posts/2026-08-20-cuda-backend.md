---
layout: gtsam-post
title: "GTSAM Goes GPU: A CUDA Backend for Nonlinear Optimization"
---

Blog post by: [Ruogu Li](https://github.com/leolrg) and [Frank Dellaert](https://dellaert.github.io/)

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/cuda-backend/cuda-optimizers.png"
    alt="Architecture of GTSAM's two CUDA optimizers: a general Levenberg-Marquardt path and a GPU-resident structure-from-motion Schur-complement path, converging on shared dense, cuDSS, and PCG GPU solvers."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Two CUDA optimization paths, one GPU solver layer. The general LM path builds a sparse Jacobian on the host, while the specialized SfM path performs projection linearization and the Schur complement on the GPU. The shared solver layer provides dense Cholesky, cuDSS, and PCG; dense Cholesky is used for the reduced SfM Schur system.</figcaption>
</figure>
<br />

* TOC
{:toc}

Over the past few weeks, we have been making the two ends of pose-graph optimization faster. [FAST-Sync]({% post_url 2026-08-12-fast-sync %}) improved the *front* of the pipeline, producing much better initial estimates before nonlinear optimization begins. Then [GTSAM PR #2661](https://github.com/borglab/gtsam/pull/2661) improved the optimization itself by enabling the exact Lie-group Jacobians for `BetweenFactor` and `PriorFactor` by default.
The exact factor Jacobian costs a few more nanoseconds to evaluate, but on the 10,000-pose `w10000` graph it cut LM time after FAST-Sync by more than half!

And now **GTSAM has a new opt-in CUDA backend for nonlinear factor-graph optimization.** [GTSAM PR #2706](https://github.com/borglab/gtsam/pull/2706) adds a general sparse CUDA Levenberg-Marquardt optimizer, a shared GPU linear-solver layer, and an even more deeply accelerated path for structure from motion (SfM).

## A general CUDA path for factor graphs

The first backend is a set of linear system solvers on the GPU, and can be used with any factor graph and *any* factor type$^*$. 
The new `gtsam::cuda::SparseLevenbergMarquardtOptimizer` accepts an ordinary `NonlinearFactorGraph` and `Values`. It compiles the graph's sparse Jacobian structure once, keeps the GPU-side allocations persistent, and reuses that structure over the Levenberg-Marquardt iterations.

To accommodate any factor type$^*$, factors continue to linearize on the CPU, while the assembled sparse Jacobian is transferred to the GPU for the linear solve. Retraction and nonlinear error evaluation also remain on the CPU. The new CUDA layer offers three complementary solver choices:

- **Matrix-free preconditioned conjugate gradient (PCG)**, with warm starts and Jacobi or block-Jacobi preconditioning.
- **Sparse direct solving with NVIDIA cuDSS**, the most consistent choice in the general pose and stereo benchmarks.
- **Dense Cholesky**, only used for SfM, see below.

One backend choice does not wins everywhere: you can choose the solver that fits your problem best.

(*) Factor graphs whose factors produce finite, unconstrained `JacobianFactor`s can use the path directly, while unsupported graphs can fall back to the standard CPU optimizer.

## The bottom-line speedups

The benchmarks in PR #2706 used an NVIDIA A100 and measured complete optimizer wall time, including optimizer construction, device setup, and every LM iteration. Across the larger problems, the speedups are substantial:

| Workload | Best general CUDA result versus CPU |
|---|---:|
| 2D pose-graph optimization | **up to 3.64× faster** |
| 3D pose-graph optimization | **up to 4.70× faster** |
| Stereo landmark SLAM / visual odometry | **up to 3.58× faster** |
| BAL SfM through the general path | **up to 6.08× faster** |

Sparse cuDSS was the most consistent solver for pose graphs, reaching roughly 3× on the larger Pose2 and Pose3 cases and about 3.5× on the larger stereo graphs. On the other hand, PCG was especially effective for SfM and selected larger or denser graphs. As usual with GPUs, *tiny* problems are dominated by setup and launch overhead and can be faster on the CPU.

## But wait, there is more: All of SfM on the GPU!

Bundle adjustment has enough regular structure to go much further. For SfM, `gtsam::cuda::SfmLevenbergMarquardtOptimizer` executes the entire computational Levenberg-Marquardt loop on the GPU:

1. projection-factor linearization;
2. Schur-complement or full-normal system construction;
3. damping and linear solve;
4. camera and landmark retraction; and
5. error evaluation.

The host only drives the LM lambda search, and the final values are downloaded once. This avoids repeatedly moving the large camera-point problem across the PCIe boundary.

The specialized SfM path still lets you choose among the three GPU solver backends. The Schur complement path can use dense Cholesky, cuDSS, or PCG; the full normal equations can use cuDSS or PCG. On the tested [BAL problems](https://grail.cs.washington.edu/projects/bal/), dense Schur Cholesky was the clear winner:

| BAL problem | Best CPU | GPU-resident dense Schur | Speedup |
|---|---:|---:|---:|
| 16 cameras | 1.069 s | 0.112 s | **9.56×** |
| 88 cameras | 3.398 s | 0.390 s | **8.72×** |
| 135 cameras | 4.438 s | 0.558 s | **7.95×** |

So the bottom line for the fully GPU-resident SfM implementation is simple: **about 8–10× faster** than the best GTSAM CPU path on these benchmarks. It used to be 20x faster, but [we have also made SfM on the CPU much faster](https://github.com/borglab/gtsam/pull/2695) :-)

## The bottom line

FAST-Sync, exact Lie-group Jacobians, and CUDA acceleration improve different parts of pose-graph optimization: FAST-Sync puts PGO In a better starting place; exact Jacobians improve individual Levenberg-Marquardt steps, cutting the number of rejected steps; and, the CUDA backend then makes every linear solve much faster.

For structure from motion we now fully support Schur complement (which just means: eliminating the points first) followed by a dense solve of the resulting camera only factor graph, completely on the GPU.

Happy optimizing!

## Further browsing

- [CUDA nonlinear optimization PR #2706](https://github.com/borglab/gtsam/pull/2706)
- [Correct Lie-group Jacobians PR #2661](https://github.com/borglab/gtsam/pull/2661)
- [Speeding up Sfm on the CPU #2661](https://github.com/borglab/gtsam/pull/2695)
- [FAST-Sync in GTSAM PR #2634](https://github.com/borglab/gtsam/pull/2634)
- [FAST-Sync blog post]({% post_url 2026-08-12-fast-sync %})

_Disclosure: AI was used to help draft this post._
