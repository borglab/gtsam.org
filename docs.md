---
layout: page
title: Docs
permalink: /docs/
---

For a a quick introduction see the tutorial on [Factor Graphs and GTSAM: A Hands-on Introduction](/tutorials/intro.html).

A more thorough introduction to the use of factor graphs in robotics is the 2017 article [Factor graphs for robot perception](https://www.cc.gatech.edu/~dellaert/pubs/Dellaert17fnt.pdf) by Frank Dellaert and Michael Kaess.

### API and Wrapper Documentation

Currently, detailed API documentation is available only for C++ via the [C++ Doxygen generated site](/doxygen/).

GTSAM comes with a python wrapper (see cython directory) and a matlab wrapper (see matlab directory), and for prototyping with GTSAM we highly recommend using one of the above. The auto-generated API documentation for python/MATLAB is limited to the number and type of input arguments, and again the [doxygen docs](/doxygen/) provide the details.

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

GTSAM was developed in the lab of [Frank Dellaert](https://dellaert.github.io) at the [Georgia Institute of Technology](http://www.gatech.edu), with the help of many contributors over the years, see [THANKS](THANKS). -->
