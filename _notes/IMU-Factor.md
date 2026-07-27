---
layout: gtsam-note
title: "The Preintegrated IMU Factor"
permalink: /notes/imu-factor/
date: 2019-05-18 14:09:48 -0400
categories: roadmap
---

GTSAM includes a state-of-the-art IMU handling scheme based on:

- Todd Lupton and Salah Sukkarieh, "Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions", TRO, 28(1):61-76, 2012.

Our implementation improves on that work using integration on the manifold, as detailed in:

- Luca Carlone, Zsolt Kira, Chris Beall, Vadim Indelman, and Frank Dellaert, "Eliminating conditionally independent sets in factor graphs: a unifying perspective based on smart factors", ICRA 2014.
- Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza, "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation", RSS 2015.

If you use this factor in academic work, please cite the publications above.

In GTSAM 4, a newer and more efficient implementation based on integrating on the `NavState` tangent space is enabled by default. To switch back to the RSS 2015 version, set `GTSAM_TANGENT_PREINTEGRATION` to `OFF`.
