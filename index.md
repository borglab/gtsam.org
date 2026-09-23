---
layout: landing
title: GTSAM 4.3
description: Built by the GTSAM community, release 4.3 brings continuous-time Gaussian processes, CUDA acceleration, certifiable optimization, GNSS, hybrid inference, and 328 runnable notebooks.
permalink: /
---

<div class="launch-hero" aria-labelledby="gtsam-launch-title">
  <div class="launch-kicker"><span>GTSAM 4.3</span><span class="launch-pulse" aria-hidden="true"></span>Stable release</div>
  <div class="hero-ghost" aria-hidden="true">ESTIMATION / OPTIMIZATION / NAVIGATION</div>
  <h1 id="gtsam-launch-title"><span>GTSAM 4.3:</span><span><em>built together.</em></span></h1>
  <div class="launch-lede">A community release shaped by researchers, engineers, and collaborators across institutions. Explore continuous-time Gaussian processes, GPU acceleration, certifiable optimization, GNSS, and hybrid inference—and meet the people behind them.</div>
  <div class="launch-actions">
    <a class="launch-button launch-button-primary" href="/get_started/">Install GTSAM 4.3 <span aria-hidden="true">&rarr;</span></a>
    <a class="launch-button" href="https://borglab.github.io/gtsam/examples/">Run the examples</a>
    <a class="launch-button" href="https://github.com/borglab/gtsam/releases/tag/4.3.0">Release notes &amp; credits</a>
  </div>
  <div class="launch-scroll-cue" aria-hidden="true"><span></span>Review the changes</div>
</div>

<div class="story-rail" aria-label="Release story progress">
  <span>4.3</span><span class="story-progress"></span><span>01</span><span>02</span><span>03</span><span>04</span><span>05</span><span>06</span><span>07</span><span>08</span><span>09</span><span>10</span>
</div>

<div class="release-intro story-manifesto">
  <div class="release-eyebrow">The people and ideas behind 4.3</div>
  <h2><span>Shared research.</span><span>Shared effort.</span><span><em>Open source.</em></span></h2>
  <div class="manifesto-note">New algorithms arrive through research collaborations. They become usable software through implementation, review, tests, documentation, and years of maintenance. This release celebrates both.</div>
</div>

<div class="story-marquee" aria-hidden="true"><span>GAUSSIAN PROCESSES&nbsp;&nbsp;•&nbsp;&nbsp;CUDA&nbsp;&nbsp;•&nbsp;&nbsp;CERTIFIABLE&nbsp;&nbsp;•&nbsp;&nbsp;GNSS&nbsp;&nbsp;•&nbsp;&nbsp;HYBRID&nbsp;&nbsp;•&nbsp;&nbsp;</span><span>GAUSSIAN PROCESSES&nbsp;&nbsp;•&nbsp;&nbsp;CUDA&nbsp;&nbsp;•&nbsp;&nbsp;CERTIFIABLE&nbsp;&nbsp;•&nbsp;&nbsp;GNSS&nbsp;&nbsp;•&nbsp;&nbsp;HYBRID&nbsp;&nbsp;•&nbsp;&nbsp;</span></div>

<div class="feature-reveal feature-panel feature-python" data-feature="01" aria-labelledby="feature-python-title">
  <div class="feature-copy">
    <span class="feature-number">01 / 10</span>
    <div class="feature-label">Python and documentation</div>
    <h2 id="feature-python-title">328 runnable notebooks and expanded Python APIs.</h2>
    <p>Runnable notebooks cover introductory factor graphs, CUDA optimization, invariant filtering, certifiable SLAM, and other modules. The Python wrappers also include a PEP 561 type marker and copy-aware APIs.</p>
    <p class="feature-credit"><strong>Made accessible by the community.</strong> <a href="https://github.com/p-zach">Porter Zach</a> and many notebook authors expanded the documentation; <a href="https://github.com/ProfFan">Fan Jiang</a>, <a href="https://github.com/DLuminary">@DLuminary</a>, <a href="https://github.com/varunagrawal">Varun Agrawal</a>, and fellow contributors broadened the language interfaces.</p>
    <div class="feature-proof"><strong>Read the equations, run the code, and inspect the result</strong><span>locally or in Colab.</span></div>
    <div class="feature-resources" aria-label="Python notebooks">
      <a href="https://borglab.github.io/gtsam/examples/"><span>Notebook index</span>All 328 notebooks</a>
      <a href="https://borglab.github.io/gtsam/customfactorexample/"><span>Python notebook</span>Custom factors</a>
      <a href="https://borglab.github.io/gtsam/fixedlagsmootherexample/"><span>Python notebook</span>Fixed-lag smoothing</a>
      <a href="https://borglab.github.io/gtsam/visualisamexample/"><span>Python notebook</span>Visual iSAM2</a>
    </div>
  </div>
  <div class="feature-visual notebook-visual" aria-label="A Python notebook combining factor graph code with an interactive result">
    <div class="notebook-top"><span></span><span></span><span></span><strong>gtsam_4_3.ipynb</strong></div>
    <div class="notebook-code"><span><b>import</b> gtsam</span><span>&nbsp;</span><span>graph = gtsam.NonlinearFactorGraph()</span><span>graph.add(<mark>measurement_factor</mark>)</span><span>&nbsp;</span><span>result = gtsam.LevenbergMarquardtOptimizer(</span><span>&nbsp;&nbsp;&nbsp;&nbsp;graph, initial</span><span>).optimize()</span></div>
    <div class="notebook-output" aria-hidden="true"><div class="output-curve"></div><span></span><span></span><span></span><span></span><span></span></div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-pgo" data-feature="02" aria-labelledby="feature-pgo-title">
  <div class="feature-copy">
    <span class="feature-number">02 / 10</span>
    <div class="feature-label">Pose-graph optimization</div>
    <h2 id="feature-pgo-title">FAST-Sync and exact Lie-group Jacobians reduce w10000 refinement from 3.041 s to 1.288 s.</h2>
    <p>FAST-Sync initializes all 10,000 poses from the graph’s 64,311 relative measurements. During refinement, <code>BetweenFactor</code> and <code>PriorFactor</code> now include the Jacobian of <code>Local</code> whenever the Lie-group traits provide it.</p>
    <p class="feature-credit"><strong>A collaborative FAST-Sync effort.</strong> Shane Holmes, Yiran Luo, Firat Taxpulat, David M. Rosen, and Frank Dellaert describe the method in the <a href="/2026/08/12/fast-sync.html">FAST-Sync article</a>.</p>
    <div class="feature-proof"><strong>2.36× faster nonlinear refinement from identical initial values</strong><span>Correct Jacobians required 10 outer LM iterations and 17 inner attempts; the legacy approximation required 23 and 42. Both reached essentially the same final objective.</span></div>
    <div class="feature-resources" aria-label="Pose-graph notebooks and evidence">
      <a href="https://borglab.github.io/gtsam/fastsyncexample/"><span>Python notebook</span>FAST-Sync tutorial</a>
      <a href="https://borglab.github.io/gtsam/fastsync/"><span>Technical notebook</span>FAST-Sync derivation</a>
      <a href="https://borglab.github.io/gtsam/pose2slamexample/"><span>Python notebook</span>Pose2 SLAM</a>
      <a href="https://github.com/borglab/gtsam/pull/2661"><span>Benchmark and PR</span>Exact Jacobians #2661</a>
    </div>
  </div>
  <div class="feature-visual posegraph-visual" role="img" aria-label="w10000 pose-graph benchmark showing FAST-Sync initialization followed by refinement with exact Lie-group Jacobians">
    <div class="visual-caption">w10000 · 10,000 poses · 64,311 factors</div>
    <div class="pgo-pipeline" aria-hidden="true">
      <div><span>01</span><strong>FAST-Sync</strong><small>24.7M → 23,454 error<br>0.638 s initialization</small></div>
      <i></i>
      <div><span>02</span><strong>Exact Jacobians</strong><small>10 LM iterations<br>17 inner attempts</small></div>
      <i></i>
      <div><span>03</span><strong>Refined</strong><small>144.910 final error<br>1.288 s refinement</small></div>
    </div>
    <div class="pgo-comparison" aria-hidden="true">
      <div><span>Exact <code>Local</code> Jacobian</span><strong>1.288 s</strong><i class="pgo-exact"></i></div>
      <div><span>Legacy approximation</span><strong>3.041 s</strong><i class="pgo-legacy"></i></div>
    </div>
    <div class="pgo-ratio" aria-hidden="true"><strong>2.36×</strong><span>refinement speedup</span></div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-alt feature-cuda" data-feature="03" aria-labelledby="feature-cuda-title">
  <div class="feature-copy">
    <span class="feature-number">03 / 10</span>
    <div class="feature-label">Experimental CUDA acceleration</div>
    <h2 id="feature-cuda-title">CUDA bundle adjustment: 4.0–4.9× speedup on three BAL datasets.</h2>
    <p>A purpose-built CUDA Levenberg–Marquardt implementation for structure from motion supports dense Cholesky, cuDSS, and matrix-free PCG linear solvers.</p>
    <p class="feature-credit"><strong>Led and implemented primarily by <a href="https://github.com/leolrg">Ruogu Li</a>.</strong> Ruogu developed the CUDA optimization work in collaboration with Frank Dellaert, bringing GPU acceleration into GTSAM.</p>
    <div class="feature-proof"><strong>Measured on BAL-16, BAL-88, and BAL-135</strong><span>August 21, 2026; Intel i7-14700F and RTX 5060 Ti; timed optimize() calls exclude data loading and optimizer construction.</span></div>
    <div class="feature-resources" aria-label="CUDA notebooks and evidence">
      <a href="https://borglab.github.io/gtsam/cudasfmlevenbergmarquardtoptimizer/"><span>Technical notebook</span>CUDA SFM optimizer</a>
      <a href="https://borglab.github.io/gtsam/sparselevenbergmarquardtoptimizer/"><span>Technical notebook</span>Sparse CUDA LM</a>
      <a href="https://borglab.github.io/gtsam/cudasfmgncoptimizer/"><span>Technical notebook</span>CUDA SFM with GNC</a>
      <a href="/2026/08/20/cuda-backend.html"><span>Benchmarks</span>CUDA backend results</a>
    </div>
  </div>
  <div class="feature-visual cuda-visual" aria-label="Benchmark bars comparing CPU and CUDA bundle adjustment performance">
    <div class="gpu-orbit" aria-hidden="true"></div>
    <div class="visual-caption">Median optimization time · lower is better</div>
    <div class="benchmark-row bal-16"><span>BAL-16</span><div><span class="cpu-bar">CPU&nbsp; 0.228s</span><span class="gpu-bar">GPU&nbsp; 0.055s</span></div></div>
    <div class="benchmark-row bal-88"><span>BAL-88</span><div><span class="cpu-bar">CPU&nbsp; 0.869s</span><span class="gpu-bar">GPU&nbsp; 0.218s</span></div></div>
    <div class="benchmark-row bal-135"><span>BAL-135</span><div><span class="cpu-bar">CPU&nbsp; 1.427s</span><span class="gpu-bar">GPU&nbsp; 0.290s</span></div></div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-alt feature-certifiable" data-feature="04" aria-labelledby="feature-certifiable-title">
  <div class="feature-copy">
    <span class="feature-number">04 / 10</span>
    <div class="feature-label">Certifiable optimization</div>
    <h2 id="feature-certifiable-title">Certifiable solutions for supported SLAM problems.</h2>
    <p>The new module converts supported factor graphs to QCQPs and solves semidefinite relaxations with a solver-independent Burer–Monteiro Riemannian Staircase.</p>
    <p class="feature-credit"><strong>Research across institutions.</strong> Developed in collaboration with <a href="https://duembgen.github.io/">Frederike Dümbgen</a> at Carnegie Mellon and <a href="https://david-m-rosen.github.io/">David M. Rosen</a> and his team at Northeastern. Contributors to the implementation include <a href="https://github.com/zhexin1904">Zhexin (Jason) Xu</a> and <a href="https://github.com/avinashresearch1">Avinash Subramanian</a>, working with Frank Dellaert.</p>
    <div class="feature-proof"><strong>Recover manifold values and inspect every rank level</strong><span>with an optimality certificate when the relaxation is tight.</span></div>
    <div class="feature-resources" aria-label="Certifiable optimization notebooks">
      <a href="https://borglab.github.io/gtsam/certifiableposegraphoptimizationpose2/"><span>Python notebook</span>Certifiable Pose2 PGO</a>
      <a href="https://borglab.github.io/gtsam/certifiableposegraphoptimizationpose3/"><span>Python notebook</span>Certifiable Pose3 PGO</a>
      <a href="https://borglab.github.io/gtsam/certifiablelandmarkslampose3/"><span>Python notebook</span>Certifiable landmark SLAM</a>
      <a href="https://borglab.github.io/gtsam/certifiablerotationaveragingrot3/"><span>Python notebook</span>Rotation averaging</a>
    </div>
  </div>
  <div class="feature-visual certificate-visual" role="img" aria-label="Increasing relaxation ranks leading to a certified globally optimal solution">
    <div class="visual-caption">Burer–Monteiro rank ladder</div>
    <div class="rank-ladder" aria-hidden="true"><span>r = 3</span><span>r = 4</span><span>r = 5</span><span class="certified-step">verified</span></div>
    <div class="certificate-seal" aria-hidden="true"><span>GLOBAL</span><strong>✓</strong><span>CERTIFIED</span></div>
    <div class="certificate-metric"><span>duality gap</span><strong>≈ 0</strong></div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-constraints" data-feature="05" aria-labelledby="feature-constraints-title">
  <div class="feature-copy">
    <span class="feature-number">05 / 10</span>
    <div class="feature-label">Nonlinear constraints</div>
    <h2 id="feature-constraints-title">Constraints are represented directly in the factor graph.</h2>
    <p>Model equalities, inequalities, variable bounds, LPs, QPs, and QCQPs alongside objective factors. Available methods include penalty, active-set, and augmented Lagrangian optimization.</p>
    <p class="feature-credit"><strong>Building the optimization foundations.</strong> The release credits <a href="https://github.com/yetongumich">@yetongumich</a>, <a href="https://github.com/zhexin1904">Zhexin Xu</a>, <a href="https://github.com/avinashresearch1">Avinash Subramanian</a>, <a href="https://github.com/ProfFan">Fan Jiang</a>, and Frank Dellaert for the combined constrained and certifiable optimization work.</p>
    <div class="feature-proof"><strong>Track objective cost and constraint violation separately</strong><span>at every optimizer iteration.</span></div>
    <div class="feature-resources" aria-label="Constrained optimization notebooks">
      <a href="https://borglab.github.io/gtsam/lpproblemexample/"><span>Python notebook</span>Linear programming</a>
      <a href="https://borglab.github.io/gtsam/qpproblemexample/"><span>Python notebook</span>Quadratic programming</a>
      <a href="https://borglab.github.io/gtsam/qcqpproblemexample/"><span>Python notebook</span>QCQP examples</a>
      <a href="https://borglab.github.io/gtsam/nonlinearequalityexample/"><span>Python notebook</span>Nonlinear equalities</a>
    </div>
  </div>
  <div class="feature-visual constraint-visual" role="img" aria-label="An unconstrained optimization path converging onto a curved constraint manifold">
    <div class="constraint-field" aria-hidden="true"><span></span><span></span><span></span></div>
    <div class="constraint-manifold" aria-hidden="true"></div>
    <div class="descent-track" aria-hidden="true"><span></span><span></span><span></span><span></span><span></span><span class="solution-dot"></span></div>
    <div class="constraint-equation" aria-hidden="true">h(x) = 0</div>
    <div class="constraint-tags"><span>Equality</span><span>Inequality</span><span>Bounds</span><span>QP / QCQP</span></div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-alt feature-navigation" data-feature="06" aria-labelledby="feature-navigation-title">
  <div class="feature-copy">
    <span class="feature-number">06 / 10</span>
    <div class="feature-label">Inertial navigation</div>
    <h2 id="feature-navigation-title">Four IMU preintegration backends.</h2>
    <p>Manifold, tangent-space, Lie-group, and Galilean formulations share one interface. GTSAM 4.3 also adds Logmap-consistent errors, exact rotating-Earth dynamics, gravity-aware factors, and expanded filtering and legged-state estimation.</p>
    <p class="feature-credit"><strong>A shared navigation effort.</strong> Contributors include <a href="https://github.com/scottiyio">@scottiyio</a>, <a href="https://github.com/jenniferoum">@jenniferoum</a>, <a href="https://github.com/rohan-bansal">@rohan-bansal</a>, <a href="https://github.com/mkielo3">@mkielo3</a>, <a href="https://github.com/nkhedekar">@nkhedekar</a>, <a href="https://github.com/arihantb2">@arihantb2</a>, <a href="https://github.com/DLuminary">@DLuminary</a>, Varun Agrawal, and Frank Dellaert.</p>
    <div class="feature-proof"><strong>NEES studies compare statistical consistency</strong><span>across the available formulations.</span></div>
    <div class="feature-resources" aria-label="Inertial navigation notebooks">
      <a href="https://borglab.github.io/gtsam/gal3imuexample/"><span>Python notebook</span>Gal3 IMU</a>
      <a href="https://borglab.github.io/gtsam/navstateimuexample/"><span>Python notebook</span>NavState IMU</a>
      <a href="https://borglab.github.io/gtsam/galileanimufactornees/"><span>NEES notebook</span>Galilean consistency</a>
      <a href="https://borglab.github.io/gtsam/navstateimupimcovariancecomparison/"><span>Python notebook</span>Covariance comparison</a>
    </div>
  </div>
  <div class="feature-visual nav-visual" role="img" aria-label="An inertial trajectory with pose frames, IMU samples, Earth rotation, and GNSS observations">
    <div class="nav-globe" aria-hidden="true"><span></span><span></span><span></span></div>
    <div class="nav-flight" aria-hidden="true"></div>
    <div class="nav-poses" aria-hidden="true"><span></span><span></span><span></span><span></span><span></span><span></span></div>
    <div class="nav-fixes" aria-hidden="true"><span></span><span></span><span></span></div>
    <div class="nav-readout"><span>Gal3</span><span>SE₂(3)</span><span>Earth rate exact</span><span>Gravity in graph</span></div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-solver" data-feature="07" aria-labelledby="feature-solver-title">
  <div class="feature-copy">
    <span class="feature-number">07 / 10</span>
    <div class="feature-label">Linear solver performance</div>
    <h2 id="feature-solver-title">Faster Linear Solvers</h2>
    <p>The new multifrontal solver retains symbolic structure and packed storage between solves. It supports partial elimination, batch-factor fast paths, parallel task scheduling, and reduced-system export.</p>
    <p class="feature-credit"><strong>Performance throughout the library.</strong> The broader inference and optimization effort includes contributions from <a href="https://github.com/ProfFan">Fan Jiang</a>, <a href="https://github.com/tzvist">@tzvist</a>, <a href="https://github.com/leolrg">Ruogu Li</a>, <a href="https://github.com/jashshah999">Jash Shah</a>, Varun Agrawal, and Frank Dellaert.</p>
    <div class="feature-proof"><strong>Reuse the same Bayes tree across LM attempts</strong><span>and back-substitute without repeating symbolic analysis.</span></div>
    <div class="feature-resources" aria-label="Multifrontal solver resources">
      <a href="https://borglab.github.io/gtsam/multifrontalsolver/"><span>Technical notebook</span>Solver design and API</a>
      <a href="https://github.com/borglab/gtsam/blob/develop/gtsam/linear/MultifrontalSolver.h"><span>C++ source</span>MultifrontalSolver</a>
    </div>
  </div>
  <div class="feature-visual solver-visual" role="img" aria-label="A Bayes tree being processed in parallel and reused across optimization iterations">
    <div class="visual-caption">Reusable Bayes tree · parallel leaf work</div>
    <div class="tree-diagram" aria-hidden="true">
      <span class="tree-link link-a"></span><span class="tree-link link-b"></span><span class="tree-link link-c"></span><span class="tree-link link-d"></span><span class="tree-link link-e"></span><span class="tree-link link-f"></span>
      <span class="tree-node tree-root">R</span><span class="tree-node tree-mid mid-a">C₁</span><span class="tree-node tree-mid mid-b">C₂</span>
      <span class="tree-node tree-leaf leaf-a">L₁</span><span class="tree-node tree-leaf leaf-b">L₂</span><span class="tree-node tree-leaf leaf-c">L₃</span><span class="tree-node tree-leaf leaf-d">L₄</span>
      <span class="reuse-loop">REUSE · UPDATE · SOLVE ↻</span>
    </div>
  </div>
</div>

<div class="feature-reveal feature-panel feature-gp" data-feature="08" aria-labelledby="feature-gp-title">
  <div class="feature-copy">
    <span class="feature-number">08 / 10</span>
    <div class="feature-label">Continuous-time Gaussian processes</div>
    <h2 id="feature-gp-title">Estimate motion between measurements.</h2>
    <p>White-noise-on-acceleration models bring continuous-time trajectory estimation into GTSAM. Incorporate asynchronous measurements, interpolate poses and velocities, and query trajectory uncertainty between the states you optimize.</p>
    <p class="feature-credit"><strong>A collaboration with the University of Toronto.</strong> <a href="https://github.com/holmesco">Connor Holmes</a> contributed the GP framework with Frank Dellaert. The research collaboration includes <a href="https://asrl.utias.utoronto.ca/~tdb/">Timothy D. Barfoot</a>, Sven Lilge, and Zi Cong Guo.</p>
    <div class="feature-resources" aria-label="Gaussian-process resources">
      <a href="https://borglab.github.io/gtsam/gaussianprocesswnoainterpolationse3/"><span>Python notebook</span>Continuous-time SE(3)</a>
      <a href="/2026/05/20/gp-ct-in-gtsam.html"><span>Research &amp; contributors</span>Gaussian processes in GTSAM</a>
    </div>
  </div>
  <figure class="feature-visual gp-visual">
    <img src="/assets/images/gp-ct/gp-trajectory.png" alt="A continuous-time Gaussian-process trajectory with discrete states and an interpolated state between them." loading="lazy" />
    <figcaption>Estimate at discrete states. Query the trajectory and its uncertainty at arbitrary times. From the <a href="/2026/05/20/gp-ct-in-gtsam.html">collaborators’ continuous-time estimation article</a>.</figcaption>
  </figure>
</div>

<div class="feature-reveal feature-panel feature-gnss" data-feature="09" aria-labelledby="feature-gnss-title">
  <div class="feature-copy">
    <span class="feature-number">09 / 10</span>
    <div class="feature-label">GNSS and sensor fusion</div>
    <h2 id="feature-gnss-title">A community effort in satellite navigation.</h2>
    <p>New factors cover pseudorange, carrier phase, RTK double differences, PPP-style measurements, Doppler, and antenna lever arms. They connect satellite observations directly to GNSS/IMU estimation in the factor graph.</p>
    <p class="feature-credit"><strong>Built by a team of community collaborators.</strong> The release credits <a href="https://github.com/inuex35">Kosuke Inoue</a>, <a href="https://github.com/masoug">Sammy Guo</a>, <a href="https://github.com/kathirgounder">Kathir Gounder</a>, <a href="https://github.com/mnissov">Morten Nissov</a>, <a href="https://github.com/scottiyio">@scottiyio</a>, and <a href="https://github.com/varunagrawal">Varun Agrawal</a>.</p>
    <div class="feature-resources" aria-label="GNSS resources">
      <a href="/2026/06/10/rtk-gnss-double-difference.html"><span>Contributor article</span>RTK double-difference factors</a>
      <a href="https://github.com/borglab/gtsam/releases/tag/4.3.0"><span>Release notes</span>GNSS additions and credits</a>
    </div>
  </div>
  <div class="feature-visual capability-notes" aria-label="GNSS capabilities">
    <div class="visual-caption">Satellite measurements in the graph</div>
    <dl>
      <dt>Range &amp; phase</dt><dd>Pseudorange and carrier-phase factors, including RTK double differences.</dd>
      <dt>Motion &amp; geometry</dt><dd>Doppler/range-rate measurements and antenna lever-arm models.</dd>
      <dt>Sensor fusion</dt><dd>GNSS/IMU coupling and a GlobalPositioner abstraction.</dd>
    </dl>
  </div>
</div>

<div class="feature-reveal feature-panel feature-hybrid" data-feature="10" aria-labelledby="feature-hybrid-title">
  <div class="feature-copy">
    <span class="feature-number">10 / 10</span>
    <div class="feature-label">Hybrid and discrete inference</div>
    <h2 id="feature-hybrid-title">Reason about alternatives alongside continuous states.</h2>
    <p>Hybrid inference combines discrete choices with continuous estimates. GTSAM 4.3 expands pruning, sampling, marginalization, smoothing, and incremental inference, with broader Python support and faster sparse discrete factors.</p>
    <p class="feature-credit"><strong>Led by <a href="https://github.com/varunagrawal">Varun Agrawal</a>.</strong> Varun drove the hybrid inference effort, with contributions from Frank Dellaert, <a href="https://github.com/ProfFan">Fan Jiang</a>, <a href="https://github.com/ywkim0606">@ywkim0606</a>, and <a href="https://github.com/arutkowski">@arutkowski</a>.</p>
    <div class="feature-resources" aria-label="Hybrid inference resources">
      <a href="https://borglab.github.io/gtsam/hybrid/"><span>User guide</span>Hybrid inference</a>
      <a href="https://borglab.github.io/gtsam/hybridsmoother/"><span>Technical notebook</span>Hybrid smoothing</a>
    </div>
  </div>
  <div class="feature-visual capability-notes" aria-label="Hybrid inference capabilities">
    <div class="visual-caption">Discrete choices, continuous estimates</div>
    <dl>
      <dt>Model alternatives</dt><dd>Represent discrete modes and data-association hypotheses alongside poses and other continuous variables.</dd>
      <dt>Update beliefs</dt><dd>Use hybrid Bayes nets, Bayes trees, and incremental inference to incorporate new measurements.</dd>
      <dt>Manage hypotheses</dt><dd>Prune, sample, and marginalize; use sparse TableFactor representations for discrete inference.</dd>
    </dl>
  </div>
</div>

<section class="release-community" aria-labelledby="release-community-title">
  <div class="release-eyebrow">Thank you to the GTSAM community</div>
  <h2 id="release-community-title">A release is more than its headline features.</h2>
  <p>GTSAM 4.3 reflects years of work on correctness, robustness, geometry, incremental inference, language bindings, documentation, and builds across platforms. Every bug report, review, example, and fix helps make the library dependable.</p>
  <p>The release notes recognize <a href="https://github.com/dellaert">Frank Dellaert</a>, <a href="https://github.com/varunagrawal">Varun Agrawal</a>, and <a href="https://github.com/ProfFan">Fan Jiang</a> for driving the release, and sustained contributions from <a href="https://github.com/talregev">@talregev</a>, <a href="https://github.com/p-zach">Porter Zach</a>, <a href="https://github.com/Gold856">@Gold856</a>, <a href="https://github.com/DLuminary">@DLuminary</a>, <a href="https://github.com/jlblancoc">José Luis Blanco</a>, <a href="https://github.com/jashshah999">Jash Shah</a>, and <a href="https://github.com/akshay-krishnan">Akshay Krishnan</a>.</p>
  <p>These highlights name only part of that community. The <a href="https://github.com/borglab/gtsam/releases/tag/4.3.0">release notes</a> credit contributors throughout the release and link their pull requests; our <a href="/about/">contributors page</a> recognizes the people who have built GTSAM over its history.</p>
</section>

<div class="release-stats" aria-label="GTSAM project facts">
  <div><strong>4.3.0</strong><span>stable release</span></div>
  <div><strong>328</strong><span>runnable notebooks</span></div>
  <div><strong>C++17</strong><span>language standard</span></div>
  <div><strong>BSD</strong><span>open source</span></div>
</div>

<div class="launch-finale">
  <div class="release-eyebrow">Explore, use, and contribute</div>
  <h2>Build on the community’s work.</h2>
  <p>Try an example, share what you learn, report a bug, or help improve the next release.</p>
  <div class="launch-actions launch-actions-center">
    <a class="launch-button launch-button-primary" href="/get_started/">Install GTSAM 4.3 <span aria-hidden="true">&rarr;</span></a>
    <a class="launch-button" href="https://borglab.github.io/gtsam/examples/">Browse examples</a>
    <a class="launch-button" href="https://github.com/borglab/gtsam">Inspect the source</a>
  </div>
</div>

<div class="launch-paths" aria-label="GTSAM documentation paths">
  <a href="/docs/"><span>01</span><strong>User guide</strong><small>Module documentation</small></a>
  <a href="https://borglab.github.io/gtsam/examples/"><span>02</span><strong>Examples</strong><small>Executable notebooks</small></a>
  <a href="/doxygen/"><span>03</span><strong>C++ reference</strong><small>C++ API documentation</small></a>
  <a href="https://github.com/borglab/gtsam/tree/develop/python"><span>04</span><strong>Python</strong><small>Python package and source</small></a>
</div>
