---
layout: page
title: Docs
permalink: /docs/
---

{% include in-progress.html %}

Currently, the only documentation available is the [C++ Doxygen generated site](/doxygen/), which has its challenges but will be updated soon.

### Notes on GTSAM

{% for note in site.notes %}
#### [{{ note.title }}]({{ note.url | prepend: site.baseurl }})
{% endfor %}

### Additional Information

There is a [`GTSAM users Google group`](https://groups.google.com/forum/#!forum/gtsam-users) for general discussion.

<!-- Read about important [`GTSAM-Concepts`](GTSAM-Concepts.md) here. A primer on GTSAM Expressions,
which support (superfast) automatic differentiation,
can be found on the [GTSAM wiki on BitBucket](https://bitbucket.org/gtborg/gtsam/wiki/Home).

See the [`INSTALL`](INSTALL.md) file for more detailed installation instructions.

GTSAM is open source under the BSD license, see the [`LICENSE`](LICENSE) and [`LICENSE.BSD`](LICENSE.BSD) files.

Please see the [`examples/`](examples) directory and the [`USAGE`](USAGE.md) file for examples on how to use GTSAM.

GTSAM was developed in the lab of [Frank Dellaert](http://www.cc.gatech.edu/~dellaert) at the [Georgia Institute of Technology](http://www.gatech.edu), with the help of many contributors over the years, see [THANKS](THANKS). -->
