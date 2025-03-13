# LQR control using factor graphs

This code is meant to provide a reference for implementing LQR control using GTSAM.  For a
discussion of the intuition behind the code, please see our corresponding [blog post](gtsam.org/2019/11/01/lqr-control).

Provided in this folder are 5 files:
* `minimum_lqr_fg.py` - A MWE of using factor graphs to solve an LQR problem.  This example was
  displayed in the [blog post](gtsam.org/2019/11/01/lqr-control).
* `dynamics_lti.py` - A module for formulating and simulating linear, time invariant (LTI) dynamics problems.
* `lqr.py` - A module for formulating and solving LQR problems.
* `example0_ricatti.py` - Example code which solves an LQR problem using the traditional dynamic,
  Discrete Algebraic Ricatti Equation method.  This code is just provided as a reference to compare
  the "standard" solution to the factor graph solution.
* `example1_factorgraph.py` - Example code which solves an LQR problem using GTSAM/factor graphs.

Please note: this code was tested on the [4.1.1 version of GTSAM on pypi](https://pypi.org/project/gtsam/4.1.1/):
```python
pip install gtsam==4.1.1
```
For versions older than 4.1.0, please use this [alternate download](https://gtsam.org/assets/code_samples/lqr_control_2020_02_12.zip).
