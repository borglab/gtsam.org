---
layout: gtsam-note
title:  "Migrating from GTSAM 3"
date:   2019-05-18 14:09:48 -0400
categories: roadmap
---

GTSAM 4 introduces several new features, most notably Expressions and a python toolbox. We also deprecated some legacy functionality and wrongly named methods, but by default the flag GTSAM_ALLOW_DEPRECATED_SINCE_V4 is enabled, allowing anyone to just pull V4 and compile. To build the python toolbox, however, you will have to explicitly disable that flag.

Also, GTSAM 4 introduces traits, a C++ technique that allows optimizing with non-GTSAM types. A significant change which will not trigger a compile error is that zero-initializing of Point2 and Point3 will be deprecated, so please be aware that this might render functions using their default constructor incorrect.

