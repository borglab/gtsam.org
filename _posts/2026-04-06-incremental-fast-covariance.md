---
layout: gtsam-post
title:  "Incremental Fast Covariance with iSAM2"
---

Authors: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

In my [last post](/2026/03/29/bayes-tree-covariance-recovery.html), I described how Steiner trees in the Bayes tree enable very fast joint covariance recovery for arbitrary sets of variables. That work focused on the batch case: you eliminate once, and then you query.

However, many GTSAM users live in the incremental world. If you are running iSAM2 for SLAM, your Bayes tree is constantly evolving as new measurements arrive and old parts of the state are relinearized. Does Steiner tree covariance recovery still hold up when the tree is changing under your feet?

The answer is yes—and it is now available in `gtsam::ISAM2` via [PR 2486](https://github.com/borglab/gtsam/pull/2486).

## 1. Incremental queries are super fast

When we use iSAM2, the state grows over time. In a long-running trajectory, the Bayes tree can become quite deep. A naive covariance query between the first pose $x_0$ and the current pose $x_t$ would seem to require traversing the entire depth of that tree.

But as we saw last time, Steiner tree recovery uses shortcut conditionals to compress long, nonbranching paths. In iSAM2, this means that even as the trajectory grows to tens of thousands of poses, the cost of querying the joint covariance between the start and the end remains nearly constant.

<figure class="center" style="width: 88%; max-width: 88%;">
  <img src="/assets/images/covariance-recovery/results-isam2-w20000.png"
    alt="iSAM2 timing on w20000 dataset"
    style="width: 100%;" />
  <figcaption>Sequential iSAM2 timing on the w20000 dataset. While the update cost (blue) spikes due to relinearization, the joint covariance query cost (orange) for $\{x_0, x_t\}$ stays in a stable few-millisecond band.</figcaption>
</figure>
<br />

In the figure above, we show the timing for the `w20000` dataset, a "Manhattan-style" 2D pose graph with over 20,000 poses created by Ed Olson. We timed every iSAM2 update and every pairwise joint covariance query $\text{Cov}(x_0, x_t)$. While the iSAM2 update cost fluctuates as the algorithm performs partial re-elimination (peaking at loop closures!), the covariance query is remarkably steady, settling into a band around 3-4 milliseconds.

## 2. Why it scales: Shortcut compression in action

This is not magic: it is a direct result of how shortcuts work in the evolving Bayes tree. For a pairwise query $\{x_0, x_t\}$, the Steiner tree is just the path connecting the two cliques. As the robot moves, that path grows longer, but almost all of it is "intermediate" support that can be compressed into a single shortcut conditional.

<figure class="center" style="width: 100%; max-width: 100%; text-align: center;">
  <img src="/assets/images/covariance-recovery/results-isam2-support.png"
    alt="Shortcut compression snapshots"
    style="width: 100%; height: auto;" />
  <figcaption>Snapshots of the compressed support for $\{x_0, x_t\}$. Even as the raw path length grows to over 1,400 cliques, the compressed Steiner tree remains a tiny structure with only two query nodes and one shortcut segment.</figcaption>
</figure>
<br />

The second figure shows snapshots of this compressed support. The labels on the edges show how many raw cliques were absorbed into a single shortcut. By step 20,000, we are compressing a path of 213 cliques into a single 6-dimensional reduced system. The actual "heavy lifting" of the covariance recovery is then performed on this tiny reduced graph, which is why the cost doesn't grow with the trajectory length.

## 3. How to use it in ISAM2

With [PR 2486](https://github.com/borglab/gtsam/pull/2486), this machinery is now integrated directly into the `ISAM2` class. You can now query joint marginals directly from your iSAM2 instance:

```cpp
ISAM2 isam;
// ... add factors and update ...
KeyVector keys = {Symbol('x', 0), Symbol('x', t)};
JointMarginal joint = isam.jointMarginalCovariance(keys);
Matrix covariance = joint.fullMatrix();
```

Under the hood, `ISAM2` uses its internal `GaussianBayesTree` and the new Steiner tree localization to give you the answer in milliseconds, even for very large graphs.

## Read more

For more details on the iSAM2 API and configuration, check out the (new) [`ISAM2` user guide](https://borglab.github.io/gtsam/isam2/).

You can also find a detailed report on the covariance recovery algorithm and the Steiner tree generalization in the [CovarianceRecovery.pdf](https://github.com/borglab/gtsam/blob/develop/doc/CovarianceRecovery.pdf).

## Summary

The combination of iSAM2's incremental updates and Steiner tree's fast covariance recovery gives us the best of both worlds: we can maintain a high-rate estimate of the state and simultaneously query the exact uncertainty of any subset of variables without breaking the real-time budget.

_Disclosure: AI was used to help draft this post and prepare the results._
