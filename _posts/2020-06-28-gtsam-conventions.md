---
layout: gtsam-post
title:  "GTSAM Geometry Conventions"
categories: gtsam.org
---
Author: [Samarth Brahmbhatt](https://samarth-robo.github.io)

This post describes the 3D geometry conventions used in GTSAM.
It also describes best practices for naming your 3D point and pose variables so that you don't get lost in the web of 
inverse transformations.

# Points
- Always name your 3D points like how you would on paper. A point $^cX$ in the camera coordinate system $c$ is named `cX`.
- 3D points use uppercase letters, 2D points use lowercase letters.

# Pose
Name your pose variables like how you would write them on paper. The pose $^wT_c$ of camera $c$ in the world coordinate frame
$w$ is named `wTc`.
In GTSAM jargon, `c` is the *pose coordinate* frame, and `w` is the *world coordinate* frame.

# Composing Poses
Math: $^oTc =~^oT_w~\cdot~^wT_c$.

GTSAM code:
```cpp
Pose3 oTw = Pose3(...);
Pose3 wTc = Pose3(...);
Pose3 oTc = oTw.compose(wTc);
```

# Transforming Points *From* Pose Coordinates
Math: $^w\homo{X} =~^wT_c~\cdot~^c\homo{X}$

GTSAM Code:
```cpp
Point3 wX = wTc.transformFrom(cX);
```

# Transforming Points *To* Pose Coordinates
Math: $^c\homo{X} =~\left(^wT_c\right)^{-1}~\cdot~^w\homo{X}$

GTSAM Code:
```cpp
Point3 cX = wTc.transformTo(wX);
```

# Cameras
The GTSAM pinhole camera classes (e.g. [`PinholeBase`](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/CalibratedCamera.cpp))
internally use `transformTo()` to transform 3D points into the camera coordinates, so you should use the pose of the camera
w.r.t. world while constructing the object:
```cpp
Pose3 wTc = Pose3(...);
SimpleCamera cam(wTc, K);
```
now you can use `cam` in the `TriangulationFactor` for example. Other factors like the
[`GenericProjectionFactor`](https://github.com/borglab/gtsam/blob/develop/gtsam/slam/ProjectionFactor.h)
also use the same convention:
$$
\begin{align*}
^{sensor}\homo{X}
&=~^{sensor}T_{world}~\cdot~^{world}\homo{X}\\
&=~^{sensor}T_{body}~\cdot~^{body}T_{world}~\cdot~^{world}\homo{X}\\
&= \left(^{world}T_{body}~\cdot~^{body}T_{sensor}\right)^{-1}~\cdot~^{world}\homo{X}
\end{align*}
$$

GTSAM Code:
```cpp
Pose3 body_T_sensor = ...
Point2 sensor_p = ...  // 2D point in the image
// in the following factor,
// Symbol('T', i) is world_T_body for the i'th frame
// Symbol('X', j) is the j'th 3D point in world coordinates i.e. world_Xj
auto f = GenericProjectionFactor<Pose3, Point3, Cal3_S2>(sensor_p, noise, Symbol('T', i), Symbol('X', j), K, body_T_sensor);
```
It will project the world 3D point $^{world}\homo{X}$ into the sensor coordinates like so:
```cpp
Pose3 world_T_sensor = world_T_body.compose(body_T_sensor);
Point3 sensor_X = world_T_sensor.transformTo(world_X);
```
and then project it to the image using instrinsics and then compare it to the detection `sensor_p`.