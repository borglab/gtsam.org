---
layout: gtsam-post
title: "GTSAM Goes GPU: A CUDA Backend for Nonlinear Optimization"
---

Blog post by: [Ruogu Li](https://github.com/leolrg) and [Frank Dellaert](https://dellaert.github.io/)

* TOC
{:toc}

Over the past few weeks, we have been making the two ends of pose-graph optimization faster. [FAST-Sync]({% post_url 2026-08-12-fast-sync %}) improved the *front* of the pipeline, producing much better initial estimates before nonlinear refinement begins. Then [GTSAM PR #2661](https://github.com/borglab/gtsam/pull/2661) improved the refinement itself by enabling the exact Lie-group Jacobians for `BetweenFactor` and `PriorFactor` by default.

That Jacobian change is a lovely example of why end-to-end performance matters more than a microbenchmark. The exact factor Jacobian costs a few more nanoseconds to evaluate, but on the 10,000-pose `w10000` graph it cut Levenberg-Marquardt refinement after FAST-Sync from 3.04 seconds to 1.29 seconds—a **2.36× speedup**—by reducing rejected steps and nonlinear iterations.

Now we are accelerating the machinery in the middle as well: **GTSAM has a new opt-in CUDA backend for nonlinear factor-graph optimization.** [GTSAM PR #2706](https://github.com/borglab/gtsam/pull/2706) adds a general sparse CUDA Levenberg-Marquardt optimizer, a shared GPU linear-solver layer, and an even more deeply accelerated path for structure from motion.

## A general CUDA path for factor graphs

The new `gtsam::cuda::SparseLevenbergMarquardtOptimizer` accepts an ordinary `NonlinearFactorGraph` and `Values`. It compiles the graph's sparse Jacobian structure once, keeps the GPU-side allocations persistent, and reuses that structure over the Levenberg-Marquardt iterations.

This is a deliberately general path: factors continue to linearize on the CPU, while the assembled sparse Jacobian is transferred to the GPU for the linear algebra. Retraction and nonlinear trial-error evaluation also remain on the CPU. Factor graphs whose factors produce finite, unconstrained `JacobianFactor`s can use the path directly, while unsupported graphs can fall back to the standard CPU optimizer.

The shared CUDA solver layer offers three complementary choices:

- **Dense Cholesky**, for the reduced Schur system in the specialized SfM path.
- **Sparse direct solving with NVIDIA cuDSS**, the most consistent choice in the general pose and stereo benchmarks.
- **Matrix-free preconditioned conjugate gradient (PCG)**, with warm starts and Jacobi or block-Jacobi preconditioning.

The important point is not that one backend wins everywhere—it does not—but that the optimizer can choose the representation and solver that fit the graph.

## The bottom-line speedups

The benchmarks in PR #2706 used an NVIDIA A100 and measured complete optimizer wall time, including optimizer construction, device setup, and every LM iteration. Across the larger problems, the results are substantial:

| Workload | Best general CUDA result versus CPU |
|---|---:|
| 2D pose-graph optimization | **up to 3.64× faster** |
| 3D pose-graph optimization | **up to 4.70× faster** |
| Stereo landmark SLAM / visual odometry | **up to 3.58× faster** |
| BAL structure from motion through the general path | **up to 6.08× faster** |

Sparse cuDSS was the dependable performer for pose graphs, reaching roughly 3× on the larger Pose2 and Pose3 cases and about 3.5× on the larger stereo graphs. PCG was especially effective on BAL and selected larger or denser graphs, but it is not a universal replacement for a sparse direct solve. As usual with GPUs, tiny problems are dominated by setup and launch overhead and can be faster on the CPU.

## Wait, there is more: the entire SfM loop on the GPU

Bundle adjustment has enough regular structure to go much further. For BAL-style structure from motion, `gtsam::cuda::SfmLevenbergMarquardtOptimizer` keeps essentially the entire computational Levenberg-Marquardt loop on the GPU:

1. projection-factor linearization;
2. Schur-complement or full-normal system construction;
3. damping and linear solve;
4. camera and landmark retraction; and
5. trial-error evaluation.

The host only drives the LM lambda search, and the final values are downloaded once. This avoids repeatedly moving the large camera-point problem across the PCIe boundary.

The specialized SfM path still lets you choose among the three GPU solver backends. The Schur complement can use dense Cholesky, cuDSS, or PCG; the full normal equations can use cuDSS or PCG. On the tested BAL problems, dense Schur Cholesky was the clear winner:

| BAL problem | Best CPU | GPU-resident dense Schur | Speedup |
|---|---:|---:|---:|
| 16 cameras | 1.069 s | 0.112 s | **9.56×** |
| 88 cameras | 3.398 s | 0.390 s | **8.72×** |
| 135 cameras | 4.438 s | 0.558 s | **7.95×** |

So the bottom line for the fully GPU-resident SfM implementation is simple: **about 8–10× faster** than the best GTSAM CPU path on these benchmarks.

## Opt in and choose your backend

CUDA support is experimental, opt-in, and disabled by default, so existing GTSAM builds and APIs are unchanged. PCG requires only the CUDA Toolkit; the sparse direct backend additionally requires cuDSS.

```bash
cmake -S . -B build-cuda \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_ENABLE_CUDA=ON \
  -DGTSAM_ENABLE_CUDSS=ON
cmake --build build-cuda -j
```

For a general factor graph, selecting a backend is explicit:

```cpp
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>

gtsam::cuda::SparseLevenbergMarquardtParams params;
gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
params.linear.backend = gtsam::cuda::LinearSolverType::Cudss;

gtsam::cuda::SparseLevenbergMarquardtOptimizer optimizer(
    graph, initial, params);
gtsam::Values result = optimizer.optimize();
```

For specialized SfM, the same choice is paired with either the Schur-complement or full-normal formulation. The optimizer also exposes backend-independent diagnostics for convergence, factorization or PCG work, data transfers, and per-stage timings.

## From a better start to a faster finish

FAST-Sync, exact Lie-group Jacobians, and CUDA acceleration improve different parts of the same story. FAST-Sync puts pose-graph optimization in a better basin. Exact Jacobians give Levenberg-Marquardt a more faithful local model, cutting iterations and rejected steps. The CUDA backend then makes the remaining large linear-algebra workload much faster.

The result is not a single GPU-only algorithm. It is a layered optimization stack: a general factor-graph path where the GPU accelerates sparse linear algebra, and a specialized SfM path where the structure is regular enough to keep almost the entire nonlinear loop device-resident. With dense, sparse direct, and PCG solvers available, GTSAM can now match the backend to the problem rather than forcing every graph through the same route.

## Further browsing

- [CUDA nonlinear optimization PR #2706](https://github.com/borglab/gtsam/pull/2706)
- [Correct Lie-group Jacobians PR #2661](https://github.com/borglab/gtsam/pull/2661)
- [FAST-Sync in GTSAM PR #2634](https://github.com/borglab/gtsam/pull/2634)
- [FAST-Sync blog post]({% post_url 2026-08-12-fast-sync %})

_Disclosure: AI was used to help draft this post._
