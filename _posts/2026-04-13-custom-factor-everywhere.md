---
layout: gtsam-post
title:  "CustomFactor in Python, C++, and MATLAB"
---

Authors: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

The `CustomFactor`, originally created by [Fan Jiang](https://scholar.google.com/citations?user=MwiylHEAAAAJ&hl=en), has been available in GTSAM for a while: if you have a residual in mind not available in the "standard" GTSAM factors, `CustomFactor` is often the fastest way to build a factor without writing an entire new factor class.

This was already true in Python, and it was always possible in C++ as well. Now, with [PR 2491: MATLAB CustomFactor](https://github.com/borglab/gtsam/pull/2491), it is available in MATLAB too.

## 1. New examples across languages

We recently added and refreshed several examples that show the same basic idea in different settings:

- [Python 1D sensor fusion notebook](https://borglab.github.io/gtsam/customfactorexample/)
- [Python landmark-localization notebook](https://borglab.github.io/gtsam/customfactorlocalizationexample/)
- [C++ test showing direct use from a residual object](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/tests/testCustomFactor.cpp)

There is now also a MATLAB example from [PR 2491](https://github.com/borglab/gtsam/pull/2491):

- MATLAB sensor-fusion example: [`matlab/gtsam_examples/CustomFactorExample.m`](https://github.com/borglab/gtsam/blob/develop/matlab/gtsam_examples/CustomFactorExample.m)

The important point is not the specific toy problem. The point is that the workflow is now consistent across all three languages:

1. define a residual callback
2. provide Jacobians when requested
3. wrap it in a `CustomFactor`
4. add it to a normal nonlinear factor graph

## 2. Using the custom factor in C++

In C++, many users still jump immediately to writing a full subclass of `NoiseModelFactorN`. That is often the right long-term answer, but during prototyping - or when the factor is naturally expressed by a small residual object - `CustomFactor` can be much lighter.

The [new C++ test](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/tests/testCustomFactor.cpp) shows exactly that pattern: a small residual object is passed directly to `std::make_shared<CustomFactor>(...)`, and the resulting factor is used in a normal optimization problem. No extra factor class is needed.

That is especially useful when:

- you are experimenting with a new residual
- you need a factor quickly for a one-off research prototype

## 3. MATLAB now also has a CustomFactor!

With [PR 2491: MATLAB CustomFactor](https://github.com/borglab/gtsam/pull/2491), MATLAB users can now write callback-backed factors just like Python users.

That PR added:

- a MATLAB-facing `CustomFactor`
- callback registration and invocation machinery on the MATLAB side
- tests
- a MATLAB example matching the Python sensor-fusion example

This fills an important gap. For many users, MATLAB is still a useful and familiar place to try ideas, debug residuals, or teach concepts interactively. `CustomFactor` is a good fit for exactly that style of work.

## 4. A quick reminder on Jacobians in GTSAM

One subtle point often trips people up when writing custom factors: **the Jacobians in GTSAM are not “naive coordinate derivatives” in the usual vector calculus sense**.

For a variable `x`, GTSAM expects Jacobians with respect to a **local tangent perturbation** `ξ`, i.e., we define $H$ as the linear map for which

$$
e(\mathrm{retract}(x,\xi)) \approx e(x) + H \xi
$$

for small `ξ`.

On Lie-group variables such as `Pose2`, `Pose3`, `Rot3`, and so on, this is often implemented via the exponential map:

$$
x' = x \cdot \mathrm{Exp}(\xi).
$$

So the Jacobian is with respect to the **tangent coordinates of that local update**, not with respect to whatever raw coordinates you may have in mind from a standalone vector formula.

This is why hand-derived Jacobians sometimes feel “off” at first. A derivative that looks obvious in ambient coordinates may not be the derivative GTSAM wants unless it has been expressed in the local coordinates induced by `retract`.

## 5. GTSAM-aware numerical derivatives to the rescue

Because of that local-coordinate convention, numerical derivatives are often the fastest way to verify a custom factor, especially while prototyping.

Python users already had this in [`gtsam.utils.numerical_derivative`](https://github.com/borglab/gtsam/blob/develop/python/gtsam/utils/numerical_derivative.py).
Now MATLAB users do too, thanks to [PR 2493: numerical derivatives in MATLAB](https://github.com/borglab/gtsam/pull/2493).

That addition matters for two reasons:

- it gives MATLAB users the same practical Jacobian-checking workflow Python users already had
- it helps when vector-based intuition and GTSAM’s local-coordinate Jacobians do not line up immediately

It is particularly useful in MATLAB because some analytical derivative output-argument paths that work nicely in Python are not yet exposed there in the same way. Numerical derivatives are therefore the natural fallback for checking a callback-backed factor.

## 6. Recommended workflow

If you are designing a new factor, a good workflow is:

1. write the residual first as a `CustomFactor`
2. verify its Jacobians numerically
3. optimize a tiny synthetic problem
4. only then decide whether it deserves a dedicated factor class

That workflow now works well in Python, C++, and MATLAB.

## Summary

`CustomFactor` is no longer just a Python convenience. It is now a genuinely cross-language prototyping tool in GTSAM:

- Python has two new notebook examples
- C++ has a compact direct-use test
- MATLAB now supports callback-backed custom factors
- MATLAB now also has numerical derivatives to help verify Jacobians

If you have been putting off a custom factor because the full factor-class route felt heavy, this is a good time to try the lighter path first.

_Disclosure: AI was used to help draft this post._
