---
layout: landing
title: GTSAM 4.3
description: GTSAM 4.3 research contributions, papers, examples, and software documentation.
permalink: /
---

<header class="release-overview">
  <p class="release-version">Release 4.3.0</p>
  <h1>GTSAM 4.3</h1>
  <p class="release-summary">GTSAM is a C++ library for inference and optimization with factor graphs, with Python and MATLAB interfaces. Version 4.3 extends continuous-time estimation, certifiable optimization, hybrid inference, satellite navigation, and GPU-accelerated optimization.</p>
  <p>This overview describes selected contributions, their research collaborations, and resources for using the software. The <a href="https://github.com/borglab/gtsam/releases/tag/4.3.0">release notes</a> provide the full change list and contributor credits.</p>
  <nav class="release-links" aria-label="GTSAM documentation">
    <a href="/get_started/">Installation</a>
    <a href="/docs/">User guides</a>
    <a href="https://borglab.github.io/gtsam/examples/">Example notebooks</a>
    <a href="/doxygen/">C++ reference</a>
    <a href="https://github.com/borglab/gtsam/tree/4.3.0">4.3.0 source</a>
  </nav>
</header>

<nav class="release-contents" aria-labelledby="release-contents-title">
  <h2 id="release-contents-title">Release highlights</h2>
  <ol>
    <li><a href="#gaussian-processes">Continuous-time Gaussian processes</a></li>
    <li><a href="#certifiable">Certifiable estimation</a></li>
    <li><a href="#hybrid">Discrete–continuous inference</a></li>
    <li><a href="#gnss">GNSS factors and ambiguity resolution</a></li>
    <li><a href="#cuda">CUDA optimization</a></li>
    <li><a href="#pose-graphs">Pose-graph initialization and refinement</a></li>
    <li><a href="#constraints">Constrained optimization</a></li>
    <li><a href="#navigation">Inertial and legged navigation</a></li>
    <li><a href="#linear-solvers">Multifrontal linear solvers</a></li>
    <li><a href="#python">Python and documentation</a></li>
  </ol>
</nav>

<nav class="release-chapters" aria-label="Release sections">
  <div class="release-chapters-inner">
    <a href="#gaussian-processes">Gaussian processes</a>
    <a href="#certifiable">Certifiable</a>
    <a href="#hybrid">Hybrid</a>
    <a href="#gnss">GNSS</a>
    <a href="#cuda">CUDA</a>
    <a href="#pose-graphs">Pose graphs</a>
    <a href="#constraints">Constraints</a>
    <a href="#navigation">Navigation</a>
    <a href="#linear-solvers">Solvers</a>
    <a href="#python">Python</a>
  </div>
</nav>

<section class="release-section" id="gaussian-processes" aria-labelledby="gp-title">
  <h2 id="gp-title">Continuous-time Gaussian processes</h2>
  <div class="research-columns">
    <div>
      <p>The Gaussian-process framework represents continuous-time trajectories using motion priors on factor graphs. White-noise-on-acceleration (WNOA) priors and interpolation support estimation between trajectory states, including poses on SE(3).</p>
      <p class="attribution">This work is a collaboration with the University of Toronto. Connor Holmes and Frank Dellaert contributed the GTSAM implementation; the accompanying research is by Connor Holmes, Sven Lilge, Zi Cong Guo, Frank Dellaert, and Timothy D. Barfoot.</p>
      <p class="paper-reference">Holmes et al. <a href="https://arxiv.org/abs/2605.09073"><cite>Smoothing Out the Edges: Continuous-Time Estimation with Gaussian Process Motion Priors on Factor Graphs</cite></a>, 2026.</p>
      <ul class="resource-links">
        <li><a href="https://borglab.github.io/gtsam/gaussianprocesswnoainterpolationse3/">SE(3) interpolation notebook</a> · <a href="https://github.com/borglab/gtsam/blob/4.3.0/python/gtsam/examples/navigation/GaussianProcessWnoaInterpolationSE3.ipynb">Notebook source</a></li>
        <li>Source: <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/nonlinear/WnoaFactorGraph.h">WnoaFactorGraph</a> · <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/nonlinear/WnoaInterpolator.h">WnoaInterpolator</a></li>
        <li><a href="https://github.com/utiasASRL/2025-fnt-ctfg">Research examples</a> · <a href="/2026/05/20/gp-ct-in-gtsam.html">Implementation article</a></li>
      </ul>
    </div>
    <figure class="research-figure">
      <a href="/assets/images/gp-ct/cont-time-traj.png"><img src="/assets/images/gp-ct/cont-time-traj.png" alt="Continuous-time factor graph and a three-dimensional trajectory showing estimated and interpolated poses with uncertainty ellipsoids." width="3204" height="3761" loading="lazy"></a>
      <figcaption>Continuous-time trajectory estimation: factor-graph structure and interpolated SE(3) states with uncertainty. Figure from the <a href="/2026/05/20/gp-ct-in-gtsam.html">Gaussian-process article</a>; select the image for full resolution.</figcaption>
    </figure>
  </div>
</section>

<section class="release-section" id="certifiable" aria-labelledby="certifiable-title">
  <h2 id="certifiable-title">Certifiable estimation</h2>
  <p>The certifiable module provides semidefinite-relaxation and Riemannian-staircase methods for estimation problems, including rotation averaging, pose-graph optimization, and landmark SLAM. These methods can establish global optimality when the relaxation and its certificate satisfy the required conditions.</p>
  <p class="attribution">The research includes collaborations with David M. Rosen and his team at Northeastern University, and with Frederike Dümbgen at Carnegie Mellon University. The papers below describe the Certifiable Factor Graph Optimization framework and complementary work on exploiting chordal sparsity.</p>
  <p class="paper-reference">Zhexin Xu, Nikolas R. Sanderson, Hanna Jiamei Zhang, and David M. Rosen. <a href="https://arxiv.org/abs/2603.01267"><cite>Certifiable Factor Graph Optimization</cite></a>, 2026.</p>
  <p class="paper-reference">Avinash Subramanian, Connor Holmes, Timothy D. Barfoot, Frank Dellaert, and Frederike Dümbgen. <a href="https://arxiv.org/abs/2605.30617"><cite>Exploiting Chordal Sparsity for Globally Optimal Estimation with Factor Graphs</cite></a>, 2026.</p>
  <ul class="resource-links">
    <li>Notebooks: <a href="https://borglab.github.io/gtsam/certifiableposegraphoptimizationpose2/">2D pose graphs</a> · <a href="https://borglab.github.io/gtsam/certifiableposegraphoptimizationpose3/">3D pose graphs</a> · <a href="https://borglab.github.io/gtsam/certifiablelandmarkslampose3/">Landmark SLAM</a> · <a href="https://borglab.github.io/gtsam/certifiablerotationaveragingrot3/">Rotation averaging</a></li>
    <li><a href="https://github.com/borglab/gtsam/tree/4.3.0/gtsam/certifiable">Module source and technical documentation</a></li>
    <li><a href="/2026/06/03/certifiable-factor-graphs.html">Framework article</a> · <a href="/2026/06/01/icra-for-workshop.html">Chordal-sparsity research</a></li>
  </ul>
  <figure class="research-figure research-figure-wide">
    <a href="/assets/images/certifiable-factor-graphs/benchmarks.png"><img src="/assets/images/certifiable-factor-graphs/benchmarks.png" alt="Six benchmark reconstructions for pose-graph optimization, landmark SLAM, and range-aided SLAM." width="1600" height="1120" loading="lazy"></a>
    <figcaption>Benchmark problems used in Certifiable Factor Graph Optimization: pose-graph optimization, landmark SLAM, and range-aided SLAM. Figure from <a href="/2026/06/03/certifiable-factor-graphs.html">David Rosen’s framework article</a>.</figcaption>
  </figure>
</section>

<section class="release-section" id="hybrid" aria-labelledby="hybrid-title">
  <h2 id="hybrid-title">Discrete–continuous inference</h2>
  <p>Hybrid factor graphs combine discrete hypotheses with continuous states. Version 4.3 extends hybrid elimination and incremental smoothing, with pruning to manage the number of hypotheses maintained during inference.</p>
  <p class="attribution">Varun Agrawal led the hybrid-inference work, with contributions from Frank Dellaert, Fan Jiang, <a href="https://github.com/ywkim0606">@ywkim0606</a>, and <a href="https://github.com/arutkowski">@arutkowski</a>.</p>
  <p class="paper-reference">Varun Agrawal and Frank Dellaert. <a href="https://arxiv.org/abs/2601.00545"><cite>Variable Elimination in Hybrid Factor Graphs for Discrete-Continuous Inference &amp; Estimation</cite></a>, 2026.</p>
  <ul class="resource-links">
    <li><a href="https://borglab.github.io/gtsam/hybrid/">Hybrid inference guide</a> · <a href="https://borglab.github.io/gtsam/hybridsmoother/">HybridSmoother notebook</a></li>
    <li><a href="https://github.com/borglab/gtsam/tree/4.3.0/gtsam/hybrid">Module source</a></li>
  </ul>
</section>

<section class="release-section" id="gnss" aria-labelledby="gnss-title">
  <h2 id="gnss-title">GNSS factors and ambiguity resolution</h2>
  <p>New satellite-navigation components include pseudorange, carrier-phase, and Doppler factors, together with integer-ambiguity-resolution utilities. They support GNSS estimation and integration with inertial measurements in a common factor graph.</p>
  <p class="attribution">These additions were a community effort involving Kosuke Inoue, Sammy Guo, Kathir Gounder, Morten Nissov, <a href="https://github.com/scottiyio">@scottiyio</a>, and Varun Agrawal. Kosuke Inoue’s RTK-GNSS article provides an evaluation on urban driving data and links to the associated implementation.</p>
  <ul class="resource-links">
    <li><a href="https://borglab.github.io/gtsam/navigation/">Navigation guide</a></li>
    <li>Factor source: <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/navigation/PseudorangeFactor.h">Pseudorange</a> · <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/navigation/CarrierPhaseFactor.h">Carrier phase</a> · <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/navigation/DopplerFactor.h">Doppler</a></li>
    <li><a href="/2026/06/10/rtk-gnss-double-difference.html">RTK-GNSS evaluation</a> · <a href="https://github.com/inuex35/tightly-coupled-gnss-imu-fgo">Evaluation code</a> · <a href="https://github.com/taroz/PPC-Dataset">PPC dataset</a></li>
  </ul>
  <figure class="research-figure research-figure-wide">
    <a href="/assets/images/rtk-gnss/tokyo-result.png"><img src="/assets/images/rtk-gnss/tokyo-result.png" alt="Trajectory comparisons and position errors for three Tokyo driving sequences, distinguishing float and fixed GNSS solutions." width="1788" height="1455" loading="lazy"></a>
    <figcaption>RTK-GNSS results on three Tokyo driving sequences. The plots distinguish float and fixed solutions; the displayed error axis is limited to 0.5 m. Figure and evaluation details from <a href="/2026/06/10/rtk-gnss-double-difference.html">Kosuke Inoue’s article</a>.</figcaption>
  </figure>
</section>

<section class="release-section" id="cuda" aria-labelledby="cuda-title">
  <h2 id="cuda-title">CUDA optimization</h2>
  <p>The CUDA backend provides GPU-accelerated Levenberg–Marquardt optimization. The general sparse path retains factor linearization on the CPU and performs the linear solve on the GPU; the specialized structure-from-motion path also moves linearization to the GPU. Available solver configurations include cuDSS and preconditioned conjugate gradients, with a dense Cholesky option for the reduced SfM Schur system.</p>
  <p class="attribution">Ruogu Li implemented the CUDA backend, with contributions from Frank Dellaert. The benchmark article documents the hardware, solver configurations, and timing breakdowns.</p>
  <ul class="resource-links">
    <li>Notebooks: <a href="https://borglab.github.io/gtsam/sparselevenbergmarquardtoptimizer/">Sparse LM</a> · <a href="https://borglab.github.io/gtsam/cudasfmlevenbergmarquardtoptimizer/">CUDA SfM</a> · <a href="https://borglab.github.io/gtsam/cudasfmgncoptimizer/">Robust SfM with GNC</a></li>
    <li>Source: <a href="https://github.com/borglab/gtsam/tree/4.3.0/gtsam/nonlinear/cuda">Sparse optimization</a> · <a href="https://github.com/borglab/gtsam/tree/4.3.0/gtsam/sfm/cuda">Structure from motion</a></li>
    <li><a href="/2026/08/20/cuda-backend.html">Implementation and benchmarks</a> · <a href="https://github.com/borglab/gtsam/pull/2706">Backend pull request</a></li>
  </ul>
</section>

<section class="release-section" id="pose-graphs" aria-labelledby="pgo-title">
  <h2 id="pgo-title">Pose-graph initialization and refinement</h2>
  <div class="research-columns">
    <div>
      <p>FAST-Sync initializes group-synchronization problems on matrix Lie groups from relative measurements. It provides initial estimates for subsequent nonlinear refinement, including pose-graph optimization.</p>
      <p class="attribution">FAST-Sync is joint work by Shane Holmes, Yiran Luo, Firat Taxpulat, David M. Rosen, and Frank Dellaert.</p>
      <p class="paper-reference">Holmes et al. <a href="https://doi.org/10.1109/LRA.2026.3710327"><cite>FAST-Sync: A Fast Start for Group Synchronization on Any Matrix Lie Group</cite></a>. IEEE Robotics and Automation Letters, 11(9):10377–10384, 2026.</p>
      <p>A separate refinement improvement supplies exact <code>Local</code> Jacobians in <code>BetweenFactor</code> and <code>PriorFactor</code> when supported by the Lie-group traits. The <a href="https://github.com/borglab/gtsam/pull/2661">w10000 benchmark</a> compares refinement from identical initial values.</p>
      <ul class="resource-links">
        <li><a href="https://borglab.github.io/gtsam/fastsyncexample/">FAST-Sync example notebook</a> · <a href="https://borglab.github.io/gtsam/fastsync/">Derivation</a></li>
        <li><a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/slam/FastSync.h">FastSync source</a> · <a href="/2026/08/12/fast-sync.html">Research article</a></li>
      </ul>
    </div>
    <figure class="research-figure research-figure-portrait">
      <a href="/assets/images/fast-sync/fast-sync-mit-initialization.png"><img src="/assets/images/fast-sync/fast-sync-mit-initialization.png" alt="MIT pose-graph trajectories comparing spanning-tree initialization, FAST-Sync initialization, and nonlinear refinement." width="550" height="1020" loading="lazy"></a>
      <figcaption>MIT pose graph: spanning-tree initialization, FAST-Sync initialization, and nonlinear refinement. Figure from the <a href="/2026/08/12/fast-sync.html">FAST-Sync article</a>.</figcaption>
    </figure>
  </div>
</section>

<section class="release-section" id="constraints" aria-labelledby="constraints-title">
  <h2 id="constraints-title">Constrained optimization</h2>
  <div class="research-columns">
    <div>
      <p>The constrained-optimization module supports linear, quadratic, and quadratically constrained quadratic problems, as well as nonlinear equality and inequality constraints. The examples describe problem construction, feasible sets, and solver use.</p>
      <p class="attribution">Frank Dellaert and Yetong Zhang describe the QP and QCQP implementation. The release notes also credit Zhexin Xu, Avinash Subramanian, and Fan Jiang across the constrained and certifiable optimization additions.</p>
      <ul class="resource-links">
        <li><a href="https://borglab.github.io/gtsam/constrained/">Constrained-optimization guide</a></li>
        <li>Notebooks: <a href="https://borglab.github.io/gtsam/lpproblemexample/">LP</a> · <a href="https://borglab.github.io/gtsam/qpproblemexample/">QP</a> · <a href="https://borglab.github.io/gtsam/qcqpproblemexample/">QCQP</a> · <a href="https://borglab.github.io/gtsam/nonlinearequalityexample/">Nonlinear equality constraints</a></li>
        <li><a href="https://github.com/borglab/gtsam/tree/4.3.0/gtsam/constrained">Module source</a> · <a href="/2026/05/13/qp-qcqp-in-gtsam.html">QP and QCQP article</a></li>
      </ul>
    </div>
    <figure class="research-figure">
      <a href="/assets/images/qp-qcqp/qp-projection.png"><img src="/assets/images/qp-qcqp/qp-projection.png" alt="Quadratic objective contours, an equality constraint, an inequality boundary, and the constrained optimum." width="1600" height="1240" loading="lazy"></a>
      <figcaption>Quadratic-program example showing objective contours, constraints, and the solution. Figure from the <a href="/2026/05/13/qp-qcqp-in-gtsam.html">QP and QCQP article</a>; see the <a href="https://borglab.github.io/gtsam/qpproblemexample/">QP notebook</a> for the formulation.</figcaption>
    </figure>
  </div>
</section>

<section class="release-section" id="navigation" aria-labelledby="navigation-title">
  <h2 id="navigation-title">Inertial and legged navigation</h2>
  <p>The navigation module expands IMU preintegration and invariant-filtering formulations, with examples for Gal(3) and NavState-based estimation and studies of covariance consistency. Legged-estimation components support proprioceptive state estimation using inertial and contact information.</p>
  <p class="attribution">Navigation contributions include Frank Dellaert, <a href="https://github.com/scottiyio">@scottiyio</a>, <a href="https://github.com/jenniferoum">@jenniferoum</a>, Rohan Bansal, <a href="https://github.com/mkielo3">@mkielo3</a>, Nikhil Khedekar, <a href="https://github.com/arihantb2">@arihantb2</a>, <a href="https://github.com/DLuminary">@DLuminary</a>, and Varun Agrawal. The legged-estimation research is a collaboration with Seoul National University.</p>
  <p class="paper-reference">Frank Dellaert, Chiyun Noh, Varun Agrawal, and Ayoung Kim. <a href="https://arxiv.org/abs/2605.23100"><cite>Four Simple Proprioceptive Estimators for Legged Robots</cite></a>, 2026.</p>
  <ul class="resource-links">
    <li><a href="https://borglab.github.io/gtsam/navigation/">Navigation guide</a> · <a href="https://borglab.github.io/gtsam/gal3imuexample/">Gal(3) IMU notebook</a> · <a href="https://borglab.github.io/gtsam/navstateimuexample/">NavState IMU notebook</a></li>
    <li>Consistency studies: <a href="https://borglab.github.io/gtsam/galileanimufactornees/">Galilean IMU NEES</a> · <a href="https://borglab.github.io/gtsam/navstateimupimcovariancecomparison/">Preintegration covariance comparison</a></li>
    <li><a href="https://github.com/borglab/gtsam/tree/4.3.0/gtsam/navigation">Module source</a> · <a href="/2026/05/26/two-new-arxiv-papers.html">Legged-estimation research article</a></li>
  </ul>
</section>

<section class="release-section" id="linear-solvers" aria-labelledby="solvers-title">
  <h2 id="solvers-title">Multifrontal linear solvers</h2>
  <p>The multifrontal solver uses packed storage and reusable symbolic structure for repeated linear solves. Its notebook explains elimination, factorization, and how to configure the solver within an optimization workflow.</p>
  <p class="attribution">The broader performance work in 4.3 includes contributions from Frank Dellaert, Fan Jiang, <a href="https://github.com/tzvist">@tzvist</a>, Ruogu Li, Jash Shah, and Varun Agrawal. The release notes distinguish these changes from CUDA and other solver additions.</p>
  <ul class="resource-links">
    <li><a href="https://borglab.github.io/gtsam/multifrontalsolver/">Multifrontal solver notebook</a></li>
    <li>Source: <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/linear/MultifrontalSolver.h">MultifrontalSolver</a> · <a href="https://github.com/borglab/gtsam/blob/4.3.0/gtsam/linear/MultifrontalParameters.h">Parameters</a></li>
  </ul>
</section>

<section class="release-section" id="python" aria-labelledby="python-title">
  <h2 id="python-title">Python and documentation</h2>
  <p>The 4.3 documentation includes 328 runnable notebooks, covering introductory factor graphs and the new modules. Python-interface changes include a PEP 561 type marker and copy-aware APIs.</p>
  <p class="attribution">Porter Zach and the notebook authors expanded the documentation. Fan Jiang, <a href="https://github.com/DLuminary">@DLuminary</a>, Varun Agrawal, and other contributors extended the language interfaces.</p>
  <ul class="resource-links">
    <li><a href="https://borglab.github.io/gtsam/examples/">Notebook index</a> · <a href="/docs/">User guides</a> · <a href="/doxygen/">C++ API reference</a></li>
    <li>Examples: <a href="https://borglab.github.io/gtsam/customfactorexample/">Custom factors</a> · <a href="https://borglab.github.io/gtsam/fixedlagsmootherexample/">Fixed-lag smoothing</a> · <a href="https://borglab.github.io/gtsam/visualisamexample/">Visual iSAM2</a></li>
    <li><a href="https://github.com/borglab/gtsam/tree/4.3.0/python">Python source</a></li>
  </ul>
</section>

<section class="release-section release-acknowledgments" aria-labelledby="acknowledgments-title">
  <h2 id="acknowledgments-title">Acknowledgments</h2>
  <p>GTSAM 4.3 also includes substantial maintenance, testing, portability, packaging, and review work. In addition to the contributors named above, the release notes recognize sustained contributions from <a href="https://github.com/talregev">@talregev</a>, <a href="https://github.com/Gold856">@Gold856</a>, José Luis Blanco, Akshay Krishnan, and many others.</p>
  <p>The <a href="https://github.com/borglab/gtsam/releases/tag/4.3.0">complete release notes and contributor list</a> record this work in more detail. See also the <a href="/about/">project history and contributors</a>. Source links on this page refer to the 4.3.0 release; the online guides and notebooks may continue to evolve.</p>
</section>
