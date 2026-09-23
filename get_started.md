---
layout: get_started
title: Install GTSAM 4.3
description: Install GTSAM 4.3.0 for Python or C++, verify the installation, and run a first factor graph.
permalink: /get_started/
---

<section class="start-hero" aria-labelledby="start-title">
  <div class="start-hero-copy">
    <div class="start-version"> GTSAM 4.3.0 · stable release</div>
    <h1 id="start-title">Install GTSAM 4.3</h1>
    <p>Use the official Python wheels for the shortest path, or build the C++17 library from the tagged source release. Both routes below end with a working factor graph.</p>
    <div class="start-actions">
      <a class="start-button start-button-primary" href="#python">Install for Python <span aria-hidden="true">↓</span></a>
      <a class="start-button" href="#cpp">Build the C++ library <span aria-hidden="true">↓</span></a>
    </div>
  </div>
  <div class="start-hero-spec" aria-label="GTSAM 4.3 requirements and support">
    <div><span>Release</span><strong>4.3.0</strong><small>September 19, 2026</small></div>
    <div><span>Language</span><strong>C++17</strong><small>Required for source builds</small></div>
    <div><span>Python</span><strong>3.11–3.14</strong><small>Official wheels</small></div>
    <div><span>License</span><strong>BSD-3</strong><small>Research and commercial use</small></div>
  </div>
</section>

<nav class="start-jump" aria-label="On this page">
  <span>Start with</span>
  <a href="#python">Python wheel</a>
  <a href="#cpp">C++ source</a>
  <a href="#first-graph">First factor graph</a>
  <a href="#notebooks">Example notebooks</a>
  <a href="#migration">4.3 migration notes</a>
</nav>

<section class="start-section start-section-dark" id="python" aria-labelledby="python-title">
  <div class="section-heading">
    <div class="section-index">01</div>
    <div>
      <div class="section-kicker">Fastest path</div>
      <h2 id="python-title">Install the Python package</h2>
      <p>PyPI provides official GTSAM 4.3.0 wheels for CPython 3.11 through 3.14 on Linux x86-64, Linux ARM64, and macOS universal2.</p>
    </div>
  </div>
  <div class="install-grid">
    <div class="command-panel command-panel-primary">
      <div class="command-head"><span>Terminal</span><button type="button" data-copy="python-install">Copy</button></div>
      <pre id="python-install"><code>python -m pip install "gtsam==4.3.0"
python -c "from importlib.metadata import version; print(version('gtsam'))"</code></pre>
      <div class="command-result"><span>Expected version</span><code>4.3.0</code></div>
    </div>
    <div class="install-notes">
      <h3>What this installs</h3>
      <ul>
        <li>The Python API and compiled GTSAM library</li>
        <li>Core nonlinear, linear, discrete, hybrid, navigation, SLAM, SFM, constrained, and certifiable modules</li>
        <li>No local C++ compilation on supported wheel platforms</li>
      </ul>
      <p>For unreleased changes from <code>develop</code>, use <code>python -m pip install gtsam-develop</code>. Development wheels can change between builds.</p>
    </div>
  </div>
  <div class="compat-note"><strong>Windows:</strong> the 4.3.0 PyPI release does not provide Windows wheels. Build from source below, or use the community-maintained conda-forge package.</div>
</section>

<section class="start-section" id="cpp" aria-labelledby="cpp-title">
  <div class="section-heading">
    <div class="section-index">02</div>
    <div>
      <div class="section-kicker">C++ and custom builds</div>
      <h2 id="cpp-title">Build the tagged source release</h2>
      <p>Use the 4.3.0 tag for a reproducible build. GTSAM now requires C++17 and CMake 3.16 or newer.</p>
    </div>
  </div>
  <div class="build-layout">
    <div class="command-panel command-panel-light">
      <div class="command-head"><span>Terminal · Linux / macOS</span><button type="button" data-copy="cpp-install">Copy</button></div>
      <pre id="cpp-install"><code>git clone --branch 4.3.0 --depth 1 https://github.com/borglab/gtsam.git
cd gtsam
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --build build --target install</code></pre>
    </div>
    <div class="requirements-card">
      <h3>Continuously tested toolchains</h3>
      <dl>
        <div><dt>Linux</dt><dd>GCC 11, 13–15; Clang 11, 14, 16</dd></div>
        <div><dt>macOS</dt><dd>Xcode 16</dd></div>
        <div><dt>Windows</dt><dd>MSVC toolset 14.40</dd></div>
        <div><dt>Build system</dt><dd>CMake ≥ 3.16</dd></div>
      </dl>
      <p>Older C++17-capable toolchains may work but are not continuously tested.</p>
    </div>
  </div>

  <div class="build-options">
    <details open>
      <summary>Build without Boost</summary>
      <p>Boost-dependent features and Boost serialization are optional. Ordinary CMake builds enable both by default; disable both for a Boost-free core build:</p>
      <pre><code>cmake -S . -B build \
  -DGTSAM_USE_BOOST_FEATURES=OFF \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF</code></pre>
    </details>
    <details>
      <summary>Run the test suite</summary>
      <p>After configuration, build the <code>check</code> target. It is deliberately separate from installation.</p>
      <pre><code>cmake --build build --target check</code></pre>
    </details>
    <details>
      <summary>Windows with Ninja</summary>
      <p>Run from a Visual Studio Developer shell:</p>
      <pre><code>cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --build build --target install</code></pre>
    </details>
    <details>
      <summary>Use GTSAM from another CMake project</summary>
      <pre><code>find_package(GTSAM 4.3 REQUIRED)
target_link_libraries(my_program PRIVATE gtsam)</code></pre>
    </details>
  </div>
  <p class="detail-link">CUDA solvers, TBB, MKL, custom install prefixes, MATLAB, and platform details are covered on the <a href="/build/">complete build page</a>.</p>
</section>

<section class="start-section start-section-code" id="first-graph" aria-labelledby="graph-title">
  <div class="section-heading">
    <div class="section-index">03</div>
    <div>
      <div class="section-kicker">Verify the API</div>
      <h2 id="graph-title">Run a first factor graph</h2>
      <p>This small Pose2 problem anchors one pose, adds an odometry measurement, and estimates the second pose from deliberately perturbed initial values.</p>
    </div>
  </div>
  <div class="example-layout">
    <div class="command-panel command-panel-example">
      <div class="command-head"><span>first_graph.py</span><button type="button" data-copy="first-graph-code">Copy</button></div>
      <pre id="first-graph-code"><code>import gtsam
from gtsam.symbol_shorthand import X

graph = gtsam.NonlinearFactorGraph()
prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.1])
odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.1])

graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(), prior_noise))
graph.add(gtsam.BetweenFactorPose2(
    X(0), X(1), gtsam.Pose2(2.0, 0.0, 0.0), odom_noise
))

initial = gtsam.Values()
initial.insert(X(0), gtsam.Pose2(0.2, -0.1, 0.05))
initial.insert(X(1), gtsam.Pose2(2.3, 0.2, -0.05))

result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
print(result.atPose2(X(1)))</code></pre>
    </div>
    <div class="graph-explainer">
      <div class="mini-graph" aria-label="Two Pose2 variables connected by an odometry factor, with a prior on the first pose">
        <span class="mg-label mg-prior-label">prior</span>
        <span class="mg-label mg-odom-label">odometry</span>
        <i class="mg-line mg-line-prior"></i><i class="mg-line mg-line-odom"></i>
        <b class="mg-factor mg-prior"></b><b class="mg-factor mg-odom"></b>
        <b class="mg-variable mg-x0">x<sub>0</sub></b><b class="mg-variable mg-x1">x<sub>1</sub></b>
      </div>
      <ol>
        <li><strong>Variables</strong><span>Two robot poses on SE(2)</span></li>
        <li><strong>Factors</strong><span>A prior and a relative-pose measurement</span></li>
        <li><strong>Result</strong><span>The optimizer returns a <code>Values</code> estimate near (2, 0, 0)</span></li>
      </ol>
    </div>
  </div>
</section>

<section class="start-section" id="notebooks" aria-labelledby="notebooks-title">
  <div class="section-heading">
    <div class="section-index">04</div>
    <div>
      <div class="section-kicker">Runnable documentation</div>
      <h2 id="notebooks-title">Continue with a worked example</h2>
      <p>Choose the notebook closest to your problem. The rendered pages include explanations, outputs, source links, and Colab launchers.</p>
    </div>
  </div>
  <div class="notebook-grid">
    <a href="https://borglab.github.io/gtsam/pose2slamexample/"><span>SLAM · Python</span><strong>Pose2 SLAM</strong><small>Build and optimize a complete planar pose graph.</small><i>Open notebook →</i></a>
    <a href="https://borglab.github.io/gtsam/fastsyncexample/"><span>Initialization · Python</span><strong>FAST-Sync</strong><small>Initialize large pose graphs before nonlinear refinement.</small><i>Open notebook →</i></a>
    <a href="https://borglab.github.io/gtsam/galileanimufactornees/"><span>Navigation · Python</span><strong>IMU preintegration</strong><small>Compare preintegration backends using NEES.</small><i>Open notebook →</i></a>
    <a href="https://borglab.github.io/gtsam/augmentedlagrangianoptimizer/"><span>Optimization · Python</span><strong>Nonlinear constraints</strong><small>Solve equality- and inequality-constrained problems.</small><i>Open notebook →</i></a>
    <a href="https://borglab.github.io/gtsam/cudasfmlevenbergmarquardtoptimizer/"><span>SFM · CUDA</span><strong>CUDA bundle adjustment</strong><small>Run the dedicated GPU Levenberg–Marquardt path.</small><i>Open notebook →</i></a>
    <a href="https://borglab.github.io/gtsam/examples/" class="notebook-all"><span>328 notebooks</span><strong>Browse all examples</strong><small>Search the complete generated Python and C++ notebook index.</small><i>Open index →</i></a>
  </div>
</section>

<section class="start-section start-section-packages" aria-labelledby="packages-title">
  <div class="section-heading">
    <div class="section-index">05</div>
    <div>
      <div class="section-kicker">Other distribution channels</div>
      <h2 id="packages-title">Choose a package source deliberately</h2>
    </div>
  </div>
  <div class="package-table" role="table" aria-label="GTSAM package sources">
    <div class="package-row package-header" role="row"><span>Channel</span><span>Use it when</span><span>Maintained by</span><span></span></div>
    <div class="package-row" role="row"><strong>PyPI · <code>gtsam</code></strong><span>You want the stable Python API on a supported wheel platform.</span><span>GTSAM project</span><a href="https://pypi.org/project/gtsam/">Files →</a></div>
    <div class="package-row" role="row"><strong>Source tag · <code>4.3.0</code></strong><span>You need C++, MATLAB, CUDA, Windows, or custom build options.</span><span>GTSAM project</span><a href="https://github.com/borglab/gtsam/releases/tag/4.3.0">Release →</a></div>
    <div class="package-row" role="row"><strong>PyPI · <code>gtsam-develop</code></strong><span>You need an unreleased fix and can tolerate API changes.</span><span>GTSAM project</span><a href="https://pypi.org/project/gtsam-develop/">Nightlies →</a></div>
    <div class="package-row" role="row"><strong>conda-forge</strong><span>You manage a cross-platform environment with conda or mamba.</span><span>Community</span><a href="https://anaconda.org/conda-forge/gtsam">Package →</a></div>
  </div>
</section>

<section class="start-section start-section-migration" id="migration" aria-labelledby="migration-title">
  <div class="section-heading">
    <div class="section-index">06</div>
    <div>
      <div class="section-kicker">Coming from 4.2</div>
      <h2 id="migration-title">Three build and API changes to check</h2>
    </div>
  </div>
  <div class="migration-grid">
    <div><span>01</span><h3>C++17 is required</h3><p>Move downstream projects to a C++17-capable toolchain before adopting 4.3.</p></div>
    <div><span>02</span><h3>Boost is optional</h3><p>Core builds can omit Boost, but the two Boost feature flags default to <code>ON</code> in ordinary CMake builds.</p></div>
    <div><span>03</span><h3>Audit deprecated APIs</h3><p>Configure with <code>GTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF</code> to find APIs scheduled for removal after 4.3.</p></div>
  </div>
</section>

<section class="start-footer-cta" aria-label="GTSAM documentation links">
  <div>
    <span>GTSAM 4.3.0</span>
    <h2>Installation complete?</h2>
    <p>Use the release notes for compatibility details, or go directly to the generated API and notebook documentation.</p>
  </div>
  <div class="start-actions">
    <a class="start-button start-button-primary" href="https://github.com/borglab/gtsam/releases/tag/4.3.0">Read the release notes <span aria-hidden="true">↗</span></a>
    <a class="start-button" href="https://borglab.github.io/gtsam/">Open the documentation <span aria-hidden="true">↗</span></a>
  </div>
</section>
