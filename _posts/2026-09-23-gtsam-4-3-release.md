---
layout: gtsam-post
title: "GTSAM 4.3 is available"
description: "GTSAM 4.3 release highlights, community contributions, and links to the release notes and installation guide."
---

GTSAM 4.3.0 is finally available! 😃 🎉 The release extends inference and optimization with factor graphs across navigation, continuous-time estimation, constrained and certifiable optimization, and discrete–continuous inference. The [GTSAM 4.3 landing page]({{ '/' | relative_url }}) provides an illustrated overview, with links to papers, notebooks, user guides, and source code. The [GitHub release page](https://github.com/borglab/gtsam/releases/tag/4.3.0) contains the full release notes, contributor credits, and source downloads.

<figure>
  <a href="{{ '/' | relative_url }}">
    <img src="{{ '/assets/images/release-43/landing-page-snapshot.jpg' | relative_url }}" width="1280" height="720" alt="Snapshot of the GTSAM 4.3 landing page, showing the release overview and links to installation, documentation, notebooks, and source code.">
  </a>
  <figcaption>The GTSAM 4.3 landing page. Select the image to explore the release highlights and their associated research and software resources.</figcaption>
</figure>

The additions include inertial and legged navigation, Gaussian-process trajectory models, QP and QCQP support, certifiable estimation built on the QCQP framework, and GNSS factors and ambiguity resolution. FAST-Sync initialization, multifrontal linear solvers, experimental CUDA optimization, and extensions to hybrid inference broaden the available computational methods. The documentation includes 328 notebooks spanning worked examples and API guides.

These developments reflect contributions from researchers and engineers across the community. Continuous-time Gaussian processes were developed in collaboration with the University of Toronto; certifiable-estimation research includes David Rosen and his team at Northeastern University and Frederike Dümbgen at Carnegie Mellon University. Ruogu Li implemented the CUDA backend, Varun Agrawal led the hybrid-inference work, and the GNSS additions were a joint effort by several community collaborators. The landing page and release notes acknowledge these contributions alongside the maintenance, testing, packaging, documentation, and review work that supports the release.

To use the release, follow the [installation guide]({{ '/get_started/' | relative_url }}) for Python packages or a C++ source build. CUDA remains experimental and opt-in: using it from Python requires compiling both GTSAM and its Python wrapper on a CUDA-equipped machine, as described in the [CUDA build instructions]({{ '/build/' | relative_url }}#cuda-with-python).
