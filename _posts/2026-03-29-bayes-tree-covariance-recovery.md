---
layout: gtsam-post
title:  "Fast Covariance Recovery with Bayes Trees"
---

Authors: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

Covariance recovery in GTSAM has been faster than many users realize for a long time. If you ask for the marginal covariance of one variable, or the joint covariance of a pair of variables, GTSAM does not form a dense inverse of the full linearized system. Instead, it already exploits the structure of the Bayes tree to localize the computation to a small part of the factorization.

In [this PR](https://github.com/borglab/gtsam/pull/2476), Codex and I extended this same locality idea to larger joint queries as well. For joint covariance queries with more than a few variables, GTSAM can now identify the exact connected support needed in the Bayes tree, compress the nonbranching parts of that support, and avoid the older global fallback that used to dominate the cost.

## 1. Bayes trees for fast covariance recovery

<figure class="center" style="width: 88%; max-width: 88%;">
  <img src="/assets/images/covariance-recovery/bayes-tree-overview.svg"
    alt="Bayes tree with clique conditionals"
    style="width: 100%;" />
  <figcaption>A Bayes tree stores the multifrontal Gaussian factorization as clique conditionals. Covariance recovery works by extracting only the part of that structure needed by the query.</figcaption>
</figure>
<br />

A **Bayes tree** (an example is shown in the figure above) is the result of performing multifrontal matrix factorization on the Jacobian or Hessian of the system - the key computational solver method underlying GTSAM. However, a Bayes tree not just a solver data structure. It also tells us how uncertainty information is organized after elimination.

For a single queried variable, the required covariance comes from the clique containing that variable together with separator marginals supplied from its ancestors. All descendant branches away from that "rootward" path integrate to one and can be ignored. This means that the common one-variable marginal query was already local in GTSAM: we formed the joint for *only* that path and then integrate out all but the requested variable.

For a two-variable query, GTSAM was also doing something similarly structured. The support for a 2-variable query is the union of the two rootward paths from the queried cliques up to their lowest common ancestor (LCA). The ancestor clique contributes the shared coupling, and each branch contributes a "shortcut" conditional that summarizes everything below the ancestor without expanding the whole graph again. That path-based shortcut machinery had been implemented in GTSAM for years, if never explicitly described.

In summary, GTSAM already had a very efficient localized mechanism for the most common covariance queries, namely `|Q|=1` and `|Q|=2`.

## 2. Steiner trees for arbitrary queries

The situation is more difficult when a query contains more than two variables. The old pairwise story no longer gives a complete answer by itself: there can be several branch points, and the needed support is no longer just a union of two paths.

The new contribution is to generalize that pairwise support pattern exactly. For an arbitrary query set, the relevant computation lives on the **Steiner subtree** connecting the queried cliques in the Bayes tree. That subtree is the minimal connected Bayes-tree support that still carries the exact joint covariance information for the query.

This gives a clean structural picture:

- for one variable, the support is one rootward path
- for two variables, the support is the union of two paths to the lowest common ancestor
- for many variables, the support is the Steiner subtree connecting all queried cliques

The second ingredient is path compression. Once the Steiner subtree is identified, any nonbranching chain can be compressed into a single shortcut conditional. That is the natural generalization of the already-existing pairwise shortcut idea.

<figure class="center" style="width: 88%; max-width: 88%;">
  <img src="/assets/images/covariance-recovery/steiner-subtree-localization.svg"
    alt="Steiner subtree support inside a Bayes tree"
    style="width: 100%;" />
  <figcaption>The new structural step is to localize an arbitrary joint covariance query to the exact Steiner subtree connecting the queried cliques.</figcaption>
</figure>
<br />

## 3. Why this matters in practice

This distinction between the old localized cases and the new generalized case matters for performance.

For one-variable and two-variable queries, the gains are naturally modest, because those cases were already handled efficiently by the Bayes-tree machinery in GTSAM. The big change shows up when the query contains more than a few variables. That is where the older implementation had to revert to a much more expensive global re-elimination path.

With Steiner-subtree localization and path compression, larger joint queries stay much closer to the true support of the query. In practice, that means the big speedups begin once `|Q| > 3`, especially for local windows or other query sets whose support stays structurally small inside the Bayes tree.

<figure class="center" style="width: 84%; max-width: 84%;">
  <img src="/assets/images/covariance-recovery/results-ablation.png"
    alt="Benchmark showing the impact of Steiner localization on larger covariance queries"
    style="width: 100%;" />
  <figcaption>For larger local joint queries, the main improvement comes from Steiner-based support localization. Single and pairwise queries were already in the fast regime.</figcaption>
</figure>
<br />

## 4. Take away for GTSAM users

First, if you are already using `Marginals` in GTSAM, the common covariance queries were already benefiting from Bayes-tree structure. That speed was not an accident or a dense linear-algebra trick. It came from localized recovery on the Bayes tree itself.

Second, the same idea now scales much better to larger joint covariance queries. The user-facing API stays simple, but the internal support extraction is much more selective than before.

## Read more

 This post is about the algorithmic story. To see the detailed public API, see the (new) [`Marginals` user guide](https://borglab.github.io/gtsam/marginals/).

You can find a detailed report on the covariance recovery in GTSAM and the Steiner tree generalization: [CovarianceRecovery.pdf](https://github.com/borglab/gtsam/blob/develop/doc/CovarianceRecovery.pdf) - warning: it is almost entirely written by AI, even if supervised closely by me.

These techniques are very related to an earlier covariance-recovery paper Michael Kaess and myself wrote in 2009: [Covariance Recovery from a Square Root Information Matrix for Data Association](https://www.cs.cmu.edu/~kaess/pub/Kaess09ras.pdf), and that paper in turn was inspired by an [earlier paper by Gene Golub](https://apps.dtic.mil/sti/tr/pdf/ADA083193.pdf) from 1980.

_Disclosure: AI was used to help draft this post._
