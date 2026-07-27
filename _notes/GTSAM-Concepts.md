---
layout: gtsam-note
title: "GTSAM Concepts"
permalink: /notes/gtsam-concepts/
date: 2019-05-18 14:09:48 -0400
categories: roadmap
---

As discussed in [Generic Programming Techniques](http://www.boost.org/community/generic_programming.html), concepts define:

- associated types
- valid expressions
- invariants
- complexity guarantees

GTSAM is built on these ideas. Rather than relying on deep inheritance hierarchies, it uses `gtsam::traits` specializations plus compile-time concept checks to decide whether a type can participate in optimization, geometry, and inference code.

This page gives the high-level picture. For the more implementation-oriented guides in the current `gtsam` repository, see:

- [Overview in `doc/GTSAM-Concepts.md`](https://borglab.github.io/gtsam/gtsam-concepts/)
- [Creating a `Manifold`](https://borglab.github.io/gtsam/manifold/)
- [Creating a `Group`](https://borglab.github.io/gtsam/group/)
- [Creating a `LieGroup`](https://borglab.github.io/gtsam/liegroup/)
- [Creating a `MatrixLieGroup`](https://borglab.github.io/gtsam/matrixliegroup/)
- [Creating a `VectorSpace`](https://borglab.github.io/gtsam/vectorspace/)

## Testable

Nearly every concept in GTSAM depends on `Testable`. Unit tests assume that a type provides:

- `print(const std::string& s = "") const`
- `equals(const T& other, double tol = 1e-9) const`

These two methods are the common contract that makes assertions and debugging output work consistently across the library.

## Traits and Concept Checks

For types under GTSAM control, some behavior can be implemented through CRTP helper classes, but the main integration mechanism is still `gtsam::traits`.

Traits encode:

- associated types such as tangent vectors and Jacobians
- the structural category of a type
- the static functions that generic algorithms call

Concept checks then verify those requirements at compile time. For example, GTSAM uses macros such as `GTSAM_CONCEPT_MANIFOLD_INST(...)` and related checks to ensure a type actually satisfies the interface it claims to model.

This is why non-GTSAM types such as Eigen vectors can still participate in the framework: the abstraction boundary is traits, not inheritance.

## Manifold

To optimize over continuous types, GTSAM assumes they are manifolds. We are interested in differentiable manifolds: spaces that can be locally approximated at any point by a tangent space.

The central operations are:

- `retract`: maps a tangent vector at a point back onto the manifold
- `localCoordinates`: computes the tangent vector connecting one point to another

In GTSAM terms, these show up as:

- `traits<T>::Retract(p, v)`
- `traits<T>::Local(p, q)`
- `traits<T>::GetDimension(p)`

Typical associated types include:

- `TangentVector`
- `ChartJacobian`
- `ManifoldType`
- `structure_category`

Expected invariants:

- `Retract(p, Local(p, q)) == q`
- `Local(p, Retract(p, v)) == v`

This is the most fundamental geometry concept in GTSAM, and it is the one on which nonlinear optimization is built.

## Group

A [group](https://en.wikipedia.org/wiki/Group_(mathematics)) provides a composition law together with an identity element and an inverse for every element.

Key operations are:

- `traits<T>::Compose(p, q)`
- `traits<T>::Inverse(p)`
- `traits<T>::Between(p, q)`
- `traits<T>::Identity`

Expected invariants:

- `Compose(p, Inverse(p)) == Identity`
- `Compose(p, Between(p, q)) == q`
- `Between(p, q) == Compose(Inverse(p), q)`

GTSAM distinguishes between two group flavors:

- multiplicative groups, which compose with `operator*`
- additive groups, which compose with `operator+`

This distinction lets generic code work uniformly with both pose-like and vector-like objects.

## Lie Group

A Lie group is both a manifold and a group, with smooth group operations. This is the central abstraction for poses, rotations, and many robotics state variables.

Beyond the manifold and group requirements, Lie groups add:

- `Expmap`
- `Logmap`
- `AdjointMap`
- Jacobian-aware versions of composition, inversion, and relative transforms

In practice this means a Lie group type supports expressions such as:

- `traits<T>::Compose(p, q, Hq, Hp)`
- `traits<T>::Inverse(p, Hp)`
- `traits<T>::Between(p, q, Hq, Hp)`
- `traits<T>::Expmap(v, Hv)`
- `traits<T>::Logmap(p, Hp)`

In GTSAM, `Retract` and `Local` for Lie groups are often defined in terms of identity-centered charts using `Expmap` and `Logmap`. However, for optimization performance, some types provide a cheaper `ChartAtOrigin` rather than using the full exponential map directly.

Most Lie groups used in GTSAM are also matrix Lie groups, which means they have an underlying matrix representation and corresponding Lie algebra operations.

## Matrix Lie Group

A matrix Lie group is a Lie group with an `N x N` matrix representation and a corresponding Lie algebra matrix.

These types add operations such as:

- `matrix()`
- `Hat(xi)`
- `Vee(X)`

They also enable generic implementations of adjoint operations through the matrix representation. Many of GTSAM's familiar geometry types fit this model.

## Vector Space

While a vector space is mathematically also a manifold, treating it as a full chart-based object is usually unnecessary. For vector-space types, the operations become the familiar linear ones:

- `Identity == 0`
- `Inverse(p) == -p`
- `Compose(p, q) == p + q`
- `Between(p, q) == q - p`
- `Local(p, q) == q - p`
- `Retract(p, v) == p + v`

This simplifies many algorithms and is why Eigen vectors, matrices, `Point2`, and `Point3` can often be used naturally in GTSAM code.

Vector-space types also typically support:

- scalar multiplication
- `dot`
- `norm`
- runtime or compile-time dimension queries

## Implementation Model

At a high level:

- GTSAM types usually start with ordinary C++ class definitions
- `gtsam::traits` ties those types to the framework
- helper base classes can fill in common functionality for specific concepts
- concept checks verify correctness at compile time

This combination gives GTSAM two things at once:

- mathematically meaningful generic interfaces
- the ability to use external types, especially Eigen types, without forcing them into an inheritance tree

## Group Actions

Group actions are no longer just a conceptual extension in GTSAM. They are now implemented in the main repository via [`gtsam/base/GroupAction.h`](https://github.com/borglab/gtsam/blob/develop/gtsam/base/GroupAction.h), although there is not yet a dedicated prose guide for them comparable to the other `gtsam/base/doc` pages.

A group action describes how a group acts on another space. In abstract form, one cares about expressions such as:

- `q = Act(g, p)`
- `q = Act(g, p, Hp)`

where `g` is a group element and `p` lives in some space on which the group acts.

The current implementation in `GroupAction.h` formalizes this through a CRTP base class and supporting function objects. In particular, it exposes:

- left and right actions via `ActionType`
- `Orbit`, for partially applying the action at a fixed manifold point
- `Diffeomorphism`, for partially applying the action at a fixed group element
- `InducedVectorField`, for the push-forward action on vector fields
- helper checks for the left-action and right-action laws

This is especially relevant for symmetry-aware robotics constructions, equivariant filtering, and the usual pose-on-point actions that appear throughout geometry code.

## Lie Group Action

When the acting group is itself a Lie group, one also cares about derivatives with respect to both:

- the point being acted on
- the group element doing the acting

This becomes important for similarity transforms, pose actions on points, and other robotics geometry constructions. The new `GroupAction` implementation is designed with those Jacobian-aware use cases in mind.
