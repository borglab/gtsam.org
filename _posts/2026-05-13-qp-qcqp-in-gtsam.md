---
layout: gtsam-post
title:  "Quadratic Programs and QCQPs in GTSAM"
---

Author: [Frank Dellaert](https://dellaert.github.io/) and [Yetong Zhang](https://scholar.google.com/citations?user=Js_AA5IAAAAJ&hl=en)

<!-- - TOC -->
{:toc}

Quadratic programming is one of the quiet workhorses behind modern robot motion. In legged robot locomotion, QPs show up in contact-force allocation, whole-body control, and model-predictive control loops that have to respect friction, torque, and unilateral-contact constraints. In quadrotor flight, small QPs appear whenever a desired thrust vector has to be mapped onto bounded rotor speeds. These are everyday constrained optimization problems that many GTSAM users already solve somewhere else in their stack.

GTSAM now has first-class building blocks for QP and QCQP modeling in the `constrained` module. The new pieces let you express quadratic objectives, linear constraints, and quadratic constraints over direct `Vector` and `Matrix` entries in `Values`, while still living inside GTSAM's factor-graph-flavored ecosystem. There is also LP support, which is useful and closely related, but less central to the typical GTSAM user. This post therefore focuses on QP and QCQP.

## QP: Quadratic objectives with linear constraints

A QP optimizes a quadratic objective subject to linear equalities or inequalities. In GTSAM terms, that means you can combine a quadratic cost, such as one coming from a `HessianFactor` or `QpCost`, with constraints like `A x = b`, `A x <= b`, or `A x >= b`. Geometrically, the optimizer is looking for the lowest point of a bowl after slicing it by planes and half-spaces. The active constraints are the boundaries that actually determine the final solution.

<figure class="center" style="width: 100%; max-width: 820px;">
  <img src="/assets/images/qp-qcqp/qp-projection.png"
    alt="QP contour plot showing an unconstrained target projected onto a feasible line segment with an active upper-bound constraint."
    style="width: 100%;" />
  <figcaption>A small QP from the <a href="https://borglab.github.io/gtsam/qpproblemexample/">QP notebook</a>: the unconstrained quadratic minimum is infeasible, so the solution lands on the feasible segment where the upper-bound inequality is active.</figcaption>
</figure>
<br />

The two-dimensional QP example above makes these active constraints visible. The yellow point is the unconstrained target, the dashed line is an equality constraint, and the green segment is the part of that line that survives the inequalities. The optimizer yields the best target-compatible point that still satisfies all constraints. This same model describes many robotics QPs, even when the variables are contact forces, accelerations, or actuator commands rather than two plotted coordinates.

<figure class="center" style="width: 100%; max-width: 920px;">
  <img src="/assets/images/qp-qcqp/quadrotor-qp.png"
    alt="Quadrotor QP dashboard showing one saturated rotor, desired and achieved wrench bars, residuals, and upper-bound margins."
    style="width: 100%;" />
  <figcaption>A quadrotor allocation QP: one rotor hits its upper bound, while the remaining rotors choose the closest feasible wrench.</figcaption>
</figure>
<br />

The quadrotor allocation example above shows why small QPs matter in real-time systems. Given a desired thrust and body torque, the allocation matrix maps four normalized rotor thrusts into a wrench, while box constraints keep each rotor inside its physical range. The unconstrained request would push one rotor beyond its limit, so the constrained solution saturates that rotor and distributes the remaining effort across the others. This is a tiny, fixed-structure problem where warm starts and solver overhead matter.

In terms of solves, both LP and QP problems have a sparse active-set solver, which is the natural starting point for chains, grids, and SLAM-like structures, where sparsity matters. QP also has a dense active-set path for tiny warm-started fixed-structure problems, such as the control-allocation QP above. There dense linear algebra beats graph setup costs. Because QPs are constrained optimization problems, they can also be handed to any constrained optimizer, such as the augmented Lagrangian optimizer.

## QCQP: Quadratic constraints

A QCQP keeps the quadratic objective but allows the constraints themselves to be quadratic. Instead of only carving a bowl with lines or planes, a QCQP can impose boundaries such as ellipses, spheres, norm bounds, and other scalar quadratic relations. That small change is important because many robotics constraints are naturally quadratic, e.g., "being on the $SO(3)$ manifold" can be expressed as a set of quadratic constraints.

<figure class="center" style="width: 100%; max-width: 820px;">
  <img src="/assets/images/qp-qcqp/qcqp-geometry.png"
    alt="QCQP contour plot showing a target outside a unit disk and strip, with the solution at the intersection of two active quadratic constraints."
    style="width: 100%;" />
  <figcaption>A QCQP from the <a href="https://borglab.github.io/gtsam/qcqpproblemexample/">QCQP notebook</a>: the feasible set is shaped by quadratic constraints, and the solution lands where the active circular and strip boundaries meet.</figcaption>
</figure>
<br />

The QCQP geometry example above shows how much changes when constraints can curve. The target lies outside the feasible region, but the feasible region is no longer a polygonal slice of the plane. One quadratic constraint creates the circular boundary, while another bounds the squared vertical coordinate. The solution sits where those two active quadratic constraints meet. This is still an optimization problem over familiar `Values`, but the feasible set has become rich enough to express a much broader family of robotics problems.

In particular, QCQP is a common language for convex relaxation and certifiable state estimation. Many estimation problems that start as non-convex geometry can be lifted, relaxed, or bounded through quadratic objectives and quadratic constraints, and those formulations are often the bridge to certificates of global optimality. That is the exciting part we are only hinting at today: future posts will go deeper into how these ideas connect to certifiable estimators in GTSAM. Stay tuned !

## Takeaway

The takeaway is that GTSAM's constrained module now reaches beyond nonlinear least squares into optimization models robotics users already rely on. QP gives you a standard way to model constrained quadratic objectives, QCQP adds quadratic feasible sets, and the docs and notebooks (see below) provide runnable examples without requiring a separate optimization package just to get started. LP, QP, and QCQP open a path toward richer constrained estimation, control, and certifiable inference inside GTSAM.

## Additional Reading

The practical entry points are the notebooks and docs rather than a long API tour here. The [QpProblem docs](https://borglab.github.io/gtsam/qpproblem/) describe the core QP model, and the runnable [QP example notebook](https://borglab.github.io/gtsam/qpproblemexample/) shows both a two-dimensional geometry example and a quadrotor allocation example. The corresponding [QcqpProblem docs](https://borglab.github.io/gtsam/qcqpproblem/) and [QCQP example notebook](https://borglab.github.io/gtsam/qcqpproblemexample/) cover quadratic constraints. If you want the linear-programming sibling, the [LP example notebook](https://borglab.github.io/gtsam/lpproblemexample/) is the right place to start.

_Disclosure: AI was used to help draft this post and prepare the figures._
