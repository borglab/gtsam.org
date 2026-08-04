---
layout: gtsam-post
title:  "3D Gaussian Splatting meets Factor Graph SLAM"
---

Author: [Jash Shah](https://github.com/jashshah999)

<!-- - TOC -->
{:toc}

Existing 3D Gaussian Splatting SLAM systems optimize camera poses via gradient descent through a differentiable rasterizer. Some (e.g., [LoopSplat](https://arxiv.org/abs/2408.10154)) add loop closure as a separate post-hoc pose graph step, but the core pose estimation against the Gaussian map remains gradient-based with no factor graph structure. [gtsam-splatfactors](https://github.com/jashshah999/gtsam-splatfactors) takes a different approach: it expresses the rendering itself as a GTSAM factor, so that pose optimization, loop closure, and multi-sensor fusion all live natively in the same iSAM2 Bayes tree.

## The problem with gradient-descent SLAM

Systems like [SplaTAM](https://spla-tam.github.io/) and [MonoGS](https://arxiv.org/abs/2405.18468) achieve excellent rendering quality, but their pose optimization is decoupled from the global SLAM backend. When drift accumulates and a loop closure is detected, the correction must be applied through a separate pose graph that has no knowledge of the rendering objective. The photometric loss and the global consistency live in different optimization frameworks.

Factor graph SLAM unifies these. iSAM2's Bayes tree gives you O(log n) incremental updates when new constraints (odometry, loop closures, IMU) are added. The question is: can we express 3D Gaussian Splatting rendering as a factor in this framework?

## GaussianSplatFactor

The core idea is straightforward. We render the Gaussian map from a candidate camera pose using [gsplat](https://github.com/nerfstudio-project/gsplat), compute photometric residuals at sampled pixel locations, and provide Jacobians to GTSAM's nonlinear optimizer:

```python
factor = GaussianSplatFactor(
    gaussian_map=my_map,
    target_image=keyframe_rgb,
    K=intrinsics,
    pixel_indices=sampled_pixels,
    W=640, H=480,
)

# Add to GTSAM graph alongside odometry, loop closures, IMU, etc.
graph.add(factor.as_gtsam_factor(pose_key, noise_model))
```

The Jacobian is computed via central differences through the se(3) generators. We parameterize the viewmat as `(I - hat(xi)) @ viewmat0`, respecting GTSAM's right-exponential update convention (left-invariant error). This is verified against `Pose3.retract()`-based numerical derivatives to < 0.1% relative error.

## Architecture

Poses live in GTSAM (iSAM2 Bayes tree). Gaussians live in PyTorch (optimized via Adam). The two are coupled through alternating optimization: poses update in the factor graph, then Gaussians are refined against the updated poses.

```
Camera poses (Pose3)          Gaussian map (PyTorch)
        |                              |
   +----------+                   +----------+
   |  iSAM2   |<-- SplatFactor -->|  gsplat  |
   |  (GTSAM) |   (photometric    | renderer |
   +----------+    residual + J)  +----------+
        |
   Odometry factors
   Loop closure factors (DINOv2 + geometric verification)
   IMU preintegration factors
```

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/gaussian-splatting-slam/tum_desk_gt_vs_render.png"
    alt="TUM fr1/desk: ground-truth camera frame next to the same view rendered from the trained Gaussian map."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>The rendering is the measurement model. Left: TUM fr1/desk camera frame. Right: the same view rendered from the Gaussian map that the factors optimize against (trained with <code>examples/showcase_render.py</code>).</figcaption>
</figure>
<br />

Loop closures are detected automatically via DINOv2 appearance matching. When a revisit is detected, PnP with depth gives the relative pose, and a `BetweenFactorPose3` with a Cauchy robust kernel is inserted into the graph. iSAM2 propagates the correction globally.

## Results

### KITTI Odometry

We evaluated on four KITTI sequences with loop closures. The pipeline uses stereo depth (StereoSGBM) for PnP visual odometry, iSAM2 with Cauchy robust kernels for graph optimization, and DINOv2 for loop detection.

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/gaussian-splatting-slam/kitti_00.png"
    alt="KITTI sequence 00 trajectory comparison: VO drifts to 29m error, iSAM2 with loop closure reduces to 7.8m."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>KITTI sequence 00. Left: bird's-eye trajectory. Right: per-frame position error. Loop closure reduces ATE from 29m to 7.8m (73% improvement).</figcaption>
</figure>
<br />

| Sequence | Trajectory | VO ATE | iSAM2 + LC | Improvement |
|----------|-----------|--------|-----------|-------------|
| 00 | 1483m | 29.14m | 7.76m | 73% |
| 05 | 937m | 21.66m | 12.24m | 44% |
| 07 | 373m | 9.67m | 1.49m | 85% |
| 09 | 823m | 45.24m | 10.56m | 77% |

### TUM-RGBD

<figure class="center" style="width: 100%; max-width: 560px; text-align: center;">
  <img src="/assets/images/gaussian-splatting-slam/loop_closure_xyz.gif"
    alt="Animated loop closure correction on TUM fr1/xyz: VO trajectory builds up, loop closures detected, trajectory corrected."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>TUM fr1/xyz: VO trajectory accumulates drift, DINOv2 detects revisits, iSAM2 corrects the full trajectory.</figcaption>
</figure>
<br />

| Sequence | VO ATE (RMSE) | iSAM2 + LC | Improvement |
|----------|---------------|-----------|-------------|
| fr1/desk | 0.189m | 0.113m | 40% |
| fr1/xyz | 0.106m | 0.072m | 32% |
| fr1/room | 0.305m | 0.273m | 11% |

All numbers are translational RMSE and reproduce with a single command per sequence: `python examples/eval_tum_lc.py --seq fr1/desk` (the script downloads the data on first run).

## Why this matters for GTSAM

This demonstrates that GTSAM's factor graph infrastructure composes naturally with modern neural rendering. The `GaussianSplatFactor` slots in alongside standard odometry, loop closure, and IMU factors with no special treatment. Robust kernels, incremental updates, and covariance recovery all work out of the box.

More broadly, this is an existence proof that 3DGS-SLAM does not need to abandon decades of SLAM infrastructure. The rendering quality of Gaussian splatting and the global consistency guarantees of factor graph SLAM are complementary, not competing.

## Code and further browsing

- [gtsam-splatfactors](https://github.com/jashshah999/gtsam-splatfactors) (MIT licensed, pip installable)
- [gsplat](https://github.com/nerfstudio-project/gsplat) (the differentiable rasterizer)
- [SplaTAM](https://spla-tam.github.io/) (gradient-descent 3DGS-SLAM)
- [LoopSplat](https://arxiv.org/abs/2408.10154) (3DGS-SLAM with post-hoc PGO loop closure)
- [MonoGS](https://arxiv.org/abs/2405.18468) (monocular Gaussian SLAM)
- [DINOv2](https://github.com/facebookresearch/dinov2) (appearance features for loop detection)

_Disclosure: AI was used to help draft this post._
