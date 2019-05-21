---
layout: post
title:  "Contributing to GTSAM"
date:   2019-05-20 21:25:00 -0400
categories: roadmap
---

# Introduction
If you have improvements to GTSAM, send us your pull requests!

Our standard workflow is to fork GTSAM’s [official GitHub repository](https://github.com/borglab/gtsam) into your own GitHub account and then push your changes into a branch on your fork. Once you believe your code is ready to be merged into GTSAM’s primary repository, open a pull request via the GitHub website. Your code will then undergo an interactive review process and Continuous Integration (CI) tests before it is merged into GTSAM’s primary repository.

GTSAM’s CI service runs on all pull requests each time they are submitted and updated. Pull requests cannot be merged into master unless all unit tests pass on all supported platform configurations.

<!-- We would like to hear about your success stories if you’ve used GTSAM in your own projects. Please consider contributing to our GTSAM Gallery by editing doc/gallery.rst and submitting a pull request with the update! -->

# Licensing

Important note: GTSAM is an open source project licensed under extremely flexible terms intended to encourage use by anyone, for any purpose. When you make a contribution to the GTSAM project, you are agreeing to do so under those same terms.

# Testing

We are strong adherents of test-driven design and debugging. When you contribute a new feature, start with a unit test that exercises the API you want to provide, and only then write the code. If you believe there is an issue with GTSAM, please try to write a minimal unit test to reproduce the behavior, then fix it. Always write the test first.

# Issue Tracking

For complex changes, especially those that will span multiple PRs, please open a GitHub issue and solicit design feedback before you invest a lot of time in code.

Be prepared to engage in active code review on your pull requests. If a reviewer asks you for more information, that is a sign you should add more documentation to your PR.

A PR generally should not include more than 750 added or changed lines (the green +### number as reported by github), and must not include more than 1500 lines of *code*.

# Coding Conventions:

Fpr C++ we follow the [Google C++ style guide](https://google.github.io/styleguide/cppguide.html). 

For python we use pep8 formatting and ask that you resolve all pylint issues.

### Using GTSAM_EXPORT:

On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro GTSAM_EXPORT in your class or function definition.

For example:
```
class GTSAM_EXPORT MyClass { ... };

GTSAM_EXPORT myFunction();
```