---
layout: gtsam-note
title: "Contributing to GTSAM"
permalink: /notes/contributing/
date: 2019-05-20 21:25:00 -0400
categories: roadmap
---

# Introduction

If you have improvements to GTSAM, send us your pull requests.

Our standard workflow is to fork GTSAM's [official GitHub repository](https://github.com/borglab/gtsam) into your own GitHub account and then push your changes into a branch on your fork. Once you believe your code is ready to be merged into GTSAM's primary repository, open a pull request via GitHub. Your code will then go through review and CI before it is merged.

We generally follow a GitFlow-style branching model, so most work will land in a feature branch or a fix branch.

![GitFlow branching model](https://nvie.com/img/git-model@2x.png)

GTSAM's CI runs on pull requests each time they are submitted and updated. Pull requests cannot be merged into `master` unless the required tests pass.

# Licensing

GTSAM is open source under a permissive BSD license. By contributing, you agree that your changes are contributed under those same terms.

# Testing

We strongly prefer test-driven development. If you contribute a new feature, start with a unit test that exercises the API you want to provide, then write the code. If you suspect a bug, try to write a minimal reproducer first and fix that.

# Issue Tracking

For larger or multi-PR changes, open a GitHub issue first and solicit design feedback before investing heavily in implementation.

Be prepared to engage in active code review. If a reviewer asks for more context, that usually means the PR needs better explanation or documentation.

As a rough guideline, a PR should not include more than 750 changed lines unless there is a clear reason, and should not include more than 1500 lines of code.

# Coding Conventions

For C++, we follow the [Google C++ style guide](https://google.github.io/styleguide/cppguide.html).

For Python, we expect `pep8`-style formatting and clean linter output.

## Using `GTSAM_EXPORT`

On Windows it is necessary to explicitly export functions and classes that should be externally accessible. Do this with the `GTSAM_EXPORT` macro.

```cpp
class GTSAM_EXPORT MyClass { ... };

GTSAM_EXPORT void myFunction();
```
