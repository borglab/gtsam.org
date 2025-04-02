---
title:  "The Preintegrated IMU Factor"
date:   2019-05-18
---
GTSAM includes a state of the art IMU handling scheme based on

- Todd Lupton and Salah Sukkarieh, "Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions", TRO, 28(1):61-76, 2012.

Our implementation improves on this using integration on the manifold, as detailed in

- Luca Carlone, Zsolt Kira, Chris Beall, Vadim Indelman, and Frank Dellaert, "Eliminating conditionally independent sets in factor graphs: a unifying perspective based on smart factors", Int. Conf. on Robotics and Automation (ICRA), 2014.
- Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza, "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation", Robotics: Science and Systems (RSS), 2015.

If you are using the factor in academic work, please cite the publications above.

In GTSAM 4 a new and more efficient implementation, based on integrating on the NavState tangent space and detailed in docs/ImuFactor.pdf, is enabled by default. To switch to the RSS 2015 version, set the flag **GTSAM_TANGENT_PREINTEGRATION** to OFF.