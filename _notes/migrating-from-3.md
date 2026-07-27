---
layout: gtsam-note
title: "Migrating from GTSAM 3"
permalink: /notes/migrating-from-3/
date: 2019-05-18 14:09:48 -0400
categories: roadmap
---

GTSAM 4 introduces several new features, most notably Expressions and a Python toolbox. It also deprecates some older APIs and incorrectly named methods, but `GTSAM_ALLOW_DEPRECATED_SINCE_V4` is enabled by default so most existing code can continue to build during migration.

To build the Python toolbox, you must explicitly disable that flag.

GTSAM 4 also introduces traits, which makes it possible to optimize with non-GTSAM types.

One behavioral change that will not necessarily trigger a compile error is that zero-initializing `Point2` and `Point3` is deprecated. Code that relied on default construction may therefore need attention during migration.
