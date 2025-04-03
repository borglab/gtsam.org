# Documentation

For a hands-on mathematical introduction see the tutorial on [Factor Graphs and GTSAM](/Content/tutorial).

A more thorough introduction to the use of factor graphs in robotics is the 2017 article [Factor graphs for robot perception](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) by Frank Dellaert and Michael Kaess.

## API and Wrapper Documentation

Currently, detailed API documentation is available only for C++ via the [C++ Doxygen generated site](https://gtsam.org/doxygen/).

GTSAM comes with a python wrapper (see `cython` directory) and a matlab wrapper (see `matlab` directory), and for prototyping with GTSAM we highly recommend using one of the above. The auto-generated API documentation for python/MATLAB is limited to the number and type of input arguments, and again the [doxygen docs](https://gtsam.org/doxygen/) provide the details.

## Notes on GTSAM

:::{card} GTSAM Concepts
:link: /GTSAM-Concepts
^^^
Manifolds, groups, lie groups, vector spaces, testables...
:::

:::{card} The Preintegrated IMU Factor
:link: /IMU-Factor
^^^
IMU handling scheme
:::

:::{card} Migrating from GTSAM 3
:link: /Migrating-from-3
^^^
GTSAM 4 adds Expressions, a Python toolbox, and C++ traits for optimizing non-GTSAM types while deprecating some legacy features.
:::

:::{card} Contributing to GTSAM
:link: /Contributing
^^^
If you have improvements to GTSAM, send us your pull requests!
:::

## Additional Information

There is a GTSAM users [Google group](https://groups.google.com/g/gtsam-users) for general discussion.