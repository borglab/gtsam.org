---
layout: gtsam-post
title:  "Toward Centimeter-Level Positioning with GTSAM: Double-Difference Factors for RTK GNSS"
---

Author: [Kosuke Inoue](https://github.com/inuex35), independent researcher

<!-- - TOC -->
{:toc}

Global Navigation Satellite System (GNSS) positioning is foundational to autonomous driving, surveying, and precision agriculture. Standard single-point positioning achieves meter-level accuracy, but many applications need tighter estimates. Real-Time Kinematic (RTK) GNSS moves toward centimeter-level positioning by exploiting **double-difference** observations that cancel or reduce common-mode errors such as satellite clock biases and atmospheric delays.

GTSAM now includes built-in double-difference factors for both pseudorange and carrier-phase observations, contributed in [PR #2502](https://github.com/borglab/gtsam/pull/2502). This post explains the technique, walks through the new factors, and shows how they enable tightly-coupled GNSS-IMU fusion via factor graphs.

## Why Double Differencing?

A raw GNSS pseudorange measurement from a receiver $r$ to a satellite $s$ can be written as:

$$
P_r^s = \rho_r^s + c\,(\delta t_r - \delta t^s) + I_r^s + T_r^s + \epsilon_P
$$

where $\rho_r^s$ is the geometric range, $c$ is the speed of light, $\delta t_r$ and $\delta t^s$ are the receiver and satellite clock biases, $I$ and $T$ represent ionospheric and tropospheric delays, and $\epsilon_P$ is the measurement noise.

**Single differencing** between a rover receiver and a nearby base station eliminates the satellite clock bias $\delta t^s$ and greatly reduces atmospheric terms that are spatially correlated. **Double differencing** then takes the difference between two satellites (a reference satellite and a target satellite), which additionally eliminates the receiver clock bias $\delta t_r$. Using the operator $\Delta\nabla$ to denote the double difference:

$$
\Delta\nabla P = (\rho_r^i - \rho_b^i) - (\rho_r^j - \rho_b^j) + \epsilon_{\Delta\nabla P}
$$

where $i$ is the reference satellite, $j$ is the target satellite, $r$ is the rover, $b$ is the base station, and $\epsilon_{\Delta\nabla P}$ is the double-differenced noise. The result is a measurement that depends almost entirely on the rover position and geometry. A reference satellite is chosen *per constellation* (GPS, Galileo, BeiDou, QZSS, ...), typically the one with a high elevation angle and strong signal strength.

For carrier-phase observations, the double-difference of the phase measurement $\Phi$ is:

$$
\Delta\nabla \Phi = \Delta\nabla \rho + \lambda \cdot (N_{\text{ref}} - N_{\text{target}}) + \epsilon_{\Delta\nabla \Phi}
$$

where $\Phi$ is the carrier-phase observable in meters, $\lambda$ is the carrier wavelength, $N$ is the integer ambiguity, and $\epsilon_{\Delta\nabla \Phi}$ is the noise. Carrier-phase measurements are far more precise than pseudorange (millimeter-level noise vs. meter-level), so resolving these integer ambiguities is the key to centimeter-level positioning.

## What GTSAM Now Provides

GTSAM's `navigation` module now includes four double-difference factors along with shared helpers, all contributed in [PR #2502](https://github.com/borglab/gtsam/pull/2502).

For each satellite pair, the user adds a **pseudorange factor** and a **carrier-phase factor** to the graph. The pseudorange factor is a unary factor on the rover position, while the carrier-phase factor additionally connects to ambiguity variables $N$ that persist across epochs (as long as no cycle slip occurs).

The basic factors, `DoubleDifferencePseudorangeFactor` and `DoubleDifferenceCarrierPhaseFactor`, take a `Point3` antenna position in ECEF directly. The lever-arm variants, `DoubleDifferencePseudorangeFactorArm` and `DoubleDifferenceCarrierPhaseFactorArm`, take a `Pose3` in the navigation frame plus a body-frame lever arm, computing the antenna position internally. These are essential for tightly-coupled IMU fusion, where the optimized state is the vehicle pose.

A shared helper, `gnss::DoubleDifferenceData`, bundles the rover/base observations and satellite positions for a given satellite pair and provides the Sagnac-corrected geometric range model $\Delta\nabla\rho(\cdot)$ with Jacobians. This keeps the individual factors thin. The **pseudorange factor** is *unary* in the rover antenna position $\mathbf{x}$ and simply evaluates the model-minus-observation residual:

$$
e_P(\mathbf{x}) = \Delta\nabla\rho(\mathbf{x}) - \Delta\nabla\tilde{P}
$$

The **carrier-phase factor** is *ternary*, connecting the rover position $\mathbf{x}$ to the two satellite ambiguities $N_\text{ref}$ and $N_\text{target}$, and adds the $\lambda \cdot (N_\text{ref} - N_\text{target})$ term on top:

$$
\begin{aligned}
e_\Phi(\mathbf{x}, N_\text{ref}, N_\text{target})
&= \Delta\nabla\rho(\mathbf{x})
 + \lambda (N_\text{ref} - N_\text{target}) \\
&\quad - \Delta\nabla\tilde{\Phi}
\end{aligned}
$$

where $\Delta\nabla\tilde{P}$ and $\Delta\nabla\tilde{\Phi}$ are the double-differenced pseudorange and carrier-phase observations. For the lever-arm variants the state $\mathbf{x}$ is a `Pose3` and the antenna position is obtained from the pose and body-frame lever arm before the same residual is evaluated.

GTSAM treats each $N$ as a continuous variable during optimization (the "float" solution). Integer fixing is done outside the graph: at each epoch the float estimates and covariance are handed to the LAMBDA algorithm, which searches for the most likely integer vector. Validated fixes are then enforced in one of two ways (fix-and-hold): either a tight prior factor is added on the corresponding $N$ variables, or the integer values are substituted as constants inside the carrier-phase factors so that $N$ no longer appears as a variable at all. A fixed ambiguity is held until a cycle slip on that satellite invalidates it, at which point the corresponding $N$ is reset and re-estimated as a float.

## Tightly-Coupled GNSS-IMU Factor Graph

The diagram below illustrates how these factors fit into a factor graph for tightly-coupled GNSS-IMU positioning. At each epoch, the rover pose is connected to a pair of double-difference factors per reference/target satellite pair: a pseudorange factor (red), which is unary on the pose, and a carrier-phase factor (blue), which additionally connects to *both* ambiguity variables of the pair — the reference-satellite ambiguity $N_\text{ref}$ and the target-satellite ambiguity $N_\text{target}$ — which persist across epochs. IMU pre-integration factors connect consecutive poses, bridging the gap when satellite signals are blocked.

<figure class="center" style="width: 100%; max-width: 820px;">
  <img src="/assets/images/rtk-gnss/rtk-factor-graph.svg"
    alt="Factor graph for tightly-coupled GNSS-IMU positioning showing double-difference factors, ambiguity variables, and IMU factors."
    style="width: 100%;" />
  <figcaption>Factor graph for tightly-coupled GNSS-IMU RTK positioning. For each reference/target satellite pair, the double-difference pseudorange factor (red, unary on the pose) and carrier-phase factor (blue) form a pair. Each carrier-phase factor connects to the pose <em>and</em> both ambiguity variables of the pair, <em>N</em><sub>ref</sub> and <em>N</em><sub>tgt</sub>, which persist across epochs. IMU pre-integration factors (black) connect consecutive poses. The ambiguity variables are estimated as floats inside the graph and snapped to integers by LAMBDA outside the graph; once fixed they are either constrained with a tight prior or substituted as constants in the carrier-phase factors.</figcaption>
</figure>
<br />

By using the lever-arm factor variants, the rover state becomes a `Pose3` in the navigation frame, which can be directly connected to GTSAM's IMU pre-integration factors. This tightly-coupled approach uses raw satellite measurements directly. The full covariance structure between position, velocity, and biases is maintained throughout the graph, and when buildings block satellite signals in urban canyons, the IMU bridges the gap while GNSS constrains long-term drift.

In practice, achieving centimeter-level accuracy requires additional infrastructure beyond the GTSAM factors themselves: satellite selection, cycle-slip detection, multipath mitigation, and integer ambiguity resolution (typically via the LAMBDA algorithm). Our implementation uses [cssrlib-numba](https://github.com/inuex35/cssrlib-numba) for these observation-modeling tasks.

## Results on PPC-Dataset

We evaluated the tightly-coupled system on the Tokyo sequences of the [PPC-Dataset](https://github.com/taroz/PPC-Dataset), an open dataset of urban driving in Japan. Tokyo's dense urban canyons present a particularly challenging environment for GNSS positioning: tall buildings block direct line-of-sight to many satellites, and reflections produce multipath that corrupts both pseudorange and carrier-phase observations.

The evaluation uses the lever-arm DD factors together with `CombinedImuFactor`, non-holonomic constraint factors, and integer ambiguity variables resolved via LAMBDA with fix-and-hold. The graph is solved incrementally with `IncrementalFixedLagSmoother`.

<figure class="center" style="width: 100%; max-width: 820px;">
  <img src="/assets/images/rtk-gnss/tokyo-result.png"
    alt="Trajectory results on three Tokyo urban driving sequences from the PPC-Dataset."
    style="width: 100%;" />
  <figcaption>Trajectory estimation results on three Tokyo urban driving sequences from the PPC-Dataset. Top row: estimated trajectories with ground truth (gray), float solutions (red), and ambiguity-fixed solutions (green). Bottom row: trajectories colored by 3D position error relative to the ground truth, using a blue-to-red colormap (clipped at 0.5 m; see colorbar).</figcaption>
</figure>
<br />

The three runs achieve **49.5--60.8% fix rates** and **56.7--69.9% of epochs within 50 cm** error. Fixed epochs are mostly accurate to a few centimeters, but occasional incorrect fixes inflate the RMS to 0.21--0.81 m.

With GNSS and IMU alone, the estimate drifts during long GNSS outages (under overpasses or in tunnels). Adding LiDAR or wheel odometry factors on the same poses keeps the estimate stable through these gaps.

## Takeaway

GTSAM now provides native support for RTK GNSS double-difference pseudorange and carrier-phase factors, an important step toward centimeter-level positioning in full GNSS pipelines. These factors integrate naturally with the existing factor graph framework, enabling tightly-coupled multi-sensor fusion with IMU, wheel odometry, or any other GTSAM factor. The lever-arm variants make it straightforward to model the physical offset between the navigation state and the antenna position.

## Additional Reading

- The DD factors are available in GTSAM's `navigation` module ([PR #2502](https://github.com/borglab/gtsam/pull/2502))
- **[gnss-gtsam-rtk](https://github.com/inuex35/gnss-gtsam-rtk)**: A standalone RTK example using GTSAM's DD factors with ISAM2
- **[tightly-coupled-gnss-imu-fgo](https://github.com/inuex35/tightly-coupled-gnss-imu-fgo)**: Tightly-coupled GNSS-IMU factor graph optimization for urban driving
- **[PPC-Dataset](https://github.com/taroz/PPC-Dataset)**: Open dataset with multi-frequency GNSS and 100 Hz IMU data from urban environments in Japan

_Disclosure: AI was used to help draft this post._
