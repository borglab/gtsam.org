---
layout: page
date:   2019-05-18 14:09:48 -0400
categories: factor-graphs
---

To act sensibly in the world, robots need to infer knowledge about the world from their sensors, while drawing on a priori knowledge. There are many different such inference problems in robotics, but none of them have received as much attention as simultaneous localization and mapping (SLAM). Other inference problems include localization in a known environment, tracking other actors in the environment, and multi-robot versions of all of the above. More specialized problems are also of interest, e.g., calibration or long-term inertial navigation.

In the SLAM problem the goal is to localize a robot using the information coming from the robot’s sensors. In a simple case this could be a set of bearing measurements to a set of landmarks. If the landmarks’ positions are known, this comes down to a triangulation problem reminiscent of how ships navigate at sea. However, the additional wrinkle in SLAM is that we do not know the landmark map a priori, and hence we have to infer the unknown map simultaneously with localization with respect to the evolving map.

GTSAM is built upon the graphical language of factor graphs, which allows us to specify a joint density as a product of *factors*. However, they are more general in that they can be used to specify any factored function f(X) over a set of *variables* X, not just probability densities. Hence, we can also use GTSAM for other problems such as trajectory planning, inverse kinematics, and much more.

We aim to provide many different tutorials and articles on this site to introduce the power of factor graphs, For now, please refer to the 2017 article [Factor graphs for robot perception](https://www.cc.gatech.edu/~dellaert/pubs/Dellaert17fnt.pdf) by Frank Dellaert and Michael Kaess, for a thorough introduction with many example applications in robotics.
