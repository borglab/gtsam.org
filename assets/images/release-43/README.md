# Release overview figures

The figures support the technical summaries on the landing page.

- `plaza2-notebook.png`: static rendering of the final saved Plotly figure in
  [RangeSLAMExample_plaza2.ipynb](https://github.com/borglab/gtsam/blob/4.3.0/python/gtsam/examples/slam/RangeSLAMExample_plaza2.ipynb),
  comparing the initial odometry trajectory, optimized path, and landmarks.
  Data and styling are unchanged; exported at 900 × 650 CSS pixels with 2× resolution.
  The [interactive notebook](https://borglab.github.io/gtsam/rangeslamexample-plaza2/)
  is linked from the landing-page figure.
- `pose2-notebook.png`: unchanged PNG output embedded in
  [Pose2SLAMExample.ipynb](https://github.com/borglab/gtsam/blob/4.3.0/python/gtsam/examples/slam/Pose2SLAMExample.ipynb),
  plotting optimized poses and marginal covariances.
- `hybrid-bayes-tree.svg`: unchanged
  [data-association tutorial figure](https://github.com/borglab/gtsam/blob/4.3.0/gtsam/hybrid/doc/figures/DataAssociationHybridBayesTree.svg).
  The original embeds its fonts.
- `cuda-speedups.svg` and `cuda-speedups-mobile.svg`: desktop and phone layouts of the reported values in
  [the CUDA article](/2026/08/20/cuda-backend.html). General-path values are maxima
  across benchmarks; GPU-resident SfM values refer to three specific BAL
  problems. All timings are complete optimizer wall times on an NVIDIA A100.
  The reported ratios are used directly, not recomputed from rounded seconds.
- `multifrontal-chain.svg`: normal matrix and upper Cholesky factor computed
  from the unit-noise four-variable chain in
  [MultifrontalSolver.ipynb](https://github.com/borglab/gtsam/blob/4.3.0/gtsam/linear/doc/MultifrontalSolver.ipynb).
  This is a numerical illustration, not a benchmark or output of the C++ solver.

Reproduce these files using the `py312` environment (including Plotly and Kaleido):

```sh
python scripts/build_release_figures.py --gtsam-repo /path/to/gtsam
```

The script pins extraction to commit
`97b7a5e3b8edce204320397f00b502f57952fb38` (GTSAM 4.3.0).
The copied GTSAM outputs retain the accompanying `LICENSE.BSD` notice.
The five pre-existing research figures and the legged-estimation animation are
reused in place and credited in the page captions.
