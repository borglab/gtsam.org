---
layout: gtsam-post
title:  "Look Ma, No RANSAC"
categories: 
---

Author: Varun Agrawal  
email: <varunagrawal@gatech.edu>

## Introduction

Robust Error Models are powerful tools for supplementing parameter estimation algorithms with the added capabilities of modeling outliers. Parameter estimation is a fundamental tool used in many fields, especially in perception and robotics, and thus performing robust parameter estimation across a wide range of applications and scenarios is crucial to strong performance for many applications. This necessitates the need to manage outliers in our measurements, and Robust Error Models provide us the means to do so.

The properties of Robust Error Models can even help obviate the need for more complex, non-deterministic algorithms such as Random Sample Consensus (a.k.a. RANSAC), a fundamental tool for parameter estimation in many a roboticist's toolbox for years. While RANSAC has proven to work well in practice, it is not robust to outliers, necessitating the need for a substantially high number runs of RANSAC to converge to a good solution. By leveraging Robust Error Models, we advocate for more robust approaches to parameter estimation which are less sensitive to outliers, but are conceptually easy and intuitive like RANSAC. Moreover, these approaches are amenable to easy plug-and-play use in pre-existing optimization frameworks, requiring minimal changes to existing pipelines.

In this blog post, we demonstrate the capabilities of Robust Error Models, models that downweight outliers to provide better estimates of the final outliers which are less influenced by these outliers. To better motivate the benefits of Robust Error Models, take a look at the below images. We show, in order, an image, its warped form, and the recovered image from the `SE(2)` transformation estimation. The first image is from the ground truth transform, the second one is using RANSAC, the next one is using a vanilla parameter estimation approach, and the last one uses Robust Error Models. As you can see, while the vanilla optimization result is poor compared to the ground truth, the RANSAC and the Robust Error Model recover the transformation nicely, with the Robust Error Model's result being comparable to the RANSAC one.

<figure>
  <img src="/assets/images/robust_estimators/ground_truth_images.png" alt="my alt text"/>
  <figcaption>Image warping and recovery using ground truth SE(2) transform.</figcaption>
</figure>

<figure>
  <img src="/assets/images/robust_estimators/ransac_images.png" alt="my alt text"/>
  <figcaption>Image warping and recovery using RANSAC.</figcaption>
</figure>

<figure>
  <img src="/assets/images/robust_estimators/vanilla_model_images.png" alt="my alt text"/>
  <figcaption>Image warping and recovery using plain old parameter estimation. You can see tha the 3rd image does not line up correctly.</figcaption>
</figure>

<figure>
  <img src="/assets/images/robust_estimators/robust_model_images.png" alt="my alt text"/>
  <figcaption>Image warping and recovery using robust error models with parameter estimation. These results are comparable to the ones from RANSAC, demonstrating the promise of Robust Error Models.</figcaption>
</figure>

We begin by reviewing techniques for paramter estimation as outlined by the [tutorial by Zhengyou Zhang][1]. We then explain robust error models, showcasing their ease of use with factor graph based optimization on a simple `SE(2)` estimation problem, instead of on generic cone fitting problems. Finally, we show that given robust error models, we can easily combine them with RANSAC to give us very good results which supercede either approach.

## Parameter Estimation - A Short Tutorial

Given some measurements $z$, a common problem we encounter is estimating some parameters $\theta$, such that we can model the system 

\\[ f(z, \theta) = 0 \\]

This is a ubiquitous problem seen in multiple domains of perception and robotics and is referred to as `parameter estimation`.

More formally, let $p$ be the state/parameter vector of dimension _m_, containing the parameters we wish to estimate. Our measurement vector is $z$, and in general our measurements our corrupted by some zero-mean, gaussian distributed noise $\epsilon$, such that 

\\[y = z + \epsilon \\]

However, we also usually make many more observations than we have parameters, and in that case, our original formulation $f(z, \theta) = 0$ no longer holds. Thus, our problem boils down to optimizing a function $F(z, y)$, which we refer to as __the cost function__ or __the objective function__.

A standard framework to estimate the parameters is via Conic Fitting in the Least-Squares formulation. A conic can be described as

\\[Q(x, y) = Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0 \\]

conditioned on **A** and **C** not being simultaneously zero.
We can then describe our conic via the 5-vector:

\\[ p = [A, B, C, D, E]^T \\].

Now since $\vert\vert p \vert\vert^2$ can not be zero for a conic (a.k.a. a degenerate solution), we can set $\vert\vert p \vert\vert = 1$ to remove the arbitrary scale factor in the conic equation. The system equation then becomes

\\[ a_i^Tp = 0 \\]
with $\vert\vert p \vert\vert = 1$, where $a_i = [a_1, a_2, ..., a_n]^T$.

The final system of equations can then be modeled as:

\\[ Ap = 0 \\]

and the function to minimize becomes

\\[ F(p) = (Ap)^T(Ap) = p^TBp \\]

where $B = A^TA$ is a symmetric matrix. The solution of this function is the singular vector of A (or the eigenvector of B) corresponding to the smallest singular value of A (or smallest eigen value of B).

In this case, we are using the algebraic distance but this has some poor properties which gives us suboptimal results, for example different points contribute differently based on their position on the conic. To alleviate this issue, we should use the orthogonal distance as our error metric, which are invariant to transformations in Euclidean space. This is illustrated in the figure below, courtesy of [1]. Thus, our new optimization objective becomes

\\[ F(p) = \sum_{i=1}^{n} d^2_{i} \\]

where _d_ is defined as 

\\[ d = \sqrt{(x - x_0 - \Delta_x)^2 + (y - y_0 - \Delta_y)^2}  \\]

with 

\\[ \Delta_x = x_t - x_0, \Delta_y = y_t - y_0 \\]

where $(x_t, y_t)$ is the point on the conic, $(x, y)$ is our measurement, and $(x_0, y_0)$ is our coordinate frame origin (a.k.a the center of the ellipse). We refer the reader to [1] for an in-depth derivation of the above optimization objective.

![orthogonal distance in conics](/assets/images/robust_estimators/orthogonal_distance.png)

Since $d_i$ is complicated, we need to use iterative optimization procedures such as Gauss-Newton, Levenberg-Marquardt, etc, which come ready-to-use with GTSAM. This gives us the familiar __Ordinary Least Squares__ optimization process.

## Robust Error Models

We have derived the basic parameter optimization approach in the previous section and seen how the choice of the optimization function affects the optimality of our solution. However, another aspect we need to take into account is the effect of outliers on our optimization and final parameter values.

By default, the optimization objectives outlined above try to model all measurements equally. This means that in the presence of outliers, the optimization process might give us parameter estimates that try to fit these outliers, sacrificing accuracy on the inliers. More formally, given the *residual* $r_i$ of the $i^{th}$ datum, i.e. the difference between the $i^{th}$ observation and the fitted value, the standard least squares approach attempts to optimize the sum of all the squared residuals. This can lead to the estimated parameters being distorted due to the equal weighting of all data points. Surely, there must be a way for the objective to model inliers and outliers in a clever way based on the residual errors?

One answer to this question is a family of models known as __Robust Error Models__ or __M-estimators__. The M-estimators try to reduce the effect of outliers by replacing the squared residuals with a function of the residuals $\rho$ that weighs each residual term by some value:

\\[ p = min \sum_i^n \rho(r_i) \\]

To allow for optimization, we define $rho$ to be a symmetric, positive-definite function, thus it has a unique minimum at zero, and is less increasing than square. The benefit of this formulation is that we can now solve the above minimization objective as an __Iteratively Reweighted Least Squares__ problem.

The M-estimator of the parameter vector __p__ based on $\rho$ is the value of __p__ which solves

\\[ \sum_i \psi(r_i)\frac{\delta r_i}{\delta p_j} = 0 \\]

for $j = 1, ..., m$ (recall that the maximum/minimum of a function is at the point its derivative is equal to zero).

Here, $\psi(x) = \frac{\delta \rho(x)}{\delta x}$ is called the __influence function__, which we can use to define a __weight function__ 

\\[ w(x) = \frac{\psi{x}}{x} \\]

giving the original derivative as

\\[ \sum_i w(r_i) r_i \frac{\delta r_i}{\delta p_j} = 0 \\]

which is exactly the system of equations we obtain from iterated reweighted least squares.

In layman's terms, the influence function $\psi(x)$ measures the influence of a data point on the value of the parameter estimate. This way, the estimated parameters are intelligent to outliers and only sensitive to inliers, since the are no longer susceptible to being significantly modified by a single datum, thus making them __robust__.

### M-estimator Constraints

While M-estimators provide us with significant benefits with respect to outlier modeling, they do come with some constraints which are required to enable their use in a wide variety of optimization problems.

- The influence function should be bounded.
- The robust estimator should be unique, i.e. it should have a unique minimum. This implies that _the individual $\rho$-function is convex in variable **p**_.
- The objective should have a gradient, even when the 2nd derivative is singular.

### Common M-estimators

Below we list some of the common estimators from the literature and which are available out of the box in GTSAM. We also provide accompanying graphs of the corresponding $\rho$ function, the influence function, and the weight function in order, allowing one to visualize the differences and effects of each estimators influence function.

1. Fair
![fair m-estimator](/assets/images/robust_estimators/fair.png)

2. Huber
![huber m-estimator](/assets/images/robust_estimators/huber.png)

3. Cauchy
![cauchy m-estimator](/assets/images/robust_estimators/cauchy.png)

4. Geman-McClure
![geman-mcclure m-estimator](/assets/images/robust_estimators/gemanmcclure.png)

5. Welsch
![welsch m-estimator](/assets/images/robust_estimators/welsch.png)

6. Tukey
![tukey m-estimator](/assets/images/robust_estimators/tukey.png)

## Example with Huber Noise Model

Now it's time for the real deal. So far we've spoken about robust estimators from a theoretical perspective, with an application to cone fitting that doesn't really give intuition about the power of this technique. From a pedagogical perspective, it would be imperative to add a concrete example to demonstrate the power of a robust error model, which in this specific case is the **Huber M-estimator**, though any other provided M-estimator can be used depending on the application or preference.

For our example application, we model the case of estimating an `SE(2)` transformation between two images, a scenario commonly seen in PoseSLAM applications. For our images, we use an example image of crayon boxes retrieved from the internet, since this image lends itself to good feature detection.

![original image](/assets/images/robust_estimators/original_image.png)

To model an `SE(2)` transformation, we apply a perspective transform to the image. The transformation applied is `(x, y, theta) = (4.711, 3.702, 0.13963)`. This gives us a ground truth value to compare our methods against. The transformed image can be seen below.

![Warped image](/assets/images/robust_estimators/transformed_image.png)

We run the standard pipeline of SIFT feature extraction and FLANN+KDTree based matching to obtain a set of matches. At this point we are ready to evaluate the different methods of estimating the transformation.

To begin, we apply a straightforward optimization process based on Factor Graphs. Using GTSAM, this can be achieved in a few lines of code. We show the core part of the example below, omitting the housekeeping and data loading for brevity.

```cpp
// This is the value we wish to estimate
Pose2_ pose_expr(0);

// Set up initial values, and Factor Graph
Values initial;
ExpressionFactorGraph graph;

// provide an initial estimate which is pretty much random
initial.insert(0, Pose2(1, 1, 0.01));

// We assume the same noise model for all points (since it is the same camera)
auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

// Now we add in the factors for the measurement matches.
// Matches is a vector of 4 tuples (index1, keypoint1, index2, keypoint2)
int index_i, index_j;
Point2 p, measurement;
for (vector<tuple<int, Point2, int, Point2>>::iterator it = matches.begin();
    it != matches.end(); ++it) {

    std::tie(index_i, measurement, index_j, p) = *it;

    Point2_ predicted = transformTo(pose_expr, p);
    
    // Add the Point2 expression variable, an initial estimate, and the measurement noise.
    graph.addExpressionFactor(predicted, measurement, measurementNoise);
}

// Optimize and print basic result
Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
result.print("Final Result:\n");
```  

It is important to note that our initial estimate for the transform values is pretty arbitrary.
Running the above code give us the transform values `(6.26294, -14.6573, 0.153888)`. As you can see, these values are not very good estimates, and this error can quickly accumulate to really throw off our estimates.

Now how about we try using M-estimators via the built-in robust error models? This is a single line change as illustrated below:

```cpp
// This is the value we wish to estimate
Pose2_ pose_expr(0);

// Set up initial values, and Factor Graph
Values initial;
ExpressionFactorGraph graph;

// provide an initial estimate which is pretty much random
initial.insert(0, Pose2(1, 1, 0.01));

// We assume the same noise model for all points (since it is the same camera)
auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

// We define our robust error model here, providing the default parameter value for the estimator.
auto huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), measurementNoise);

// Now we add in the factors for the measurement matches.
// Matches is a vector of 4 tuples (index1, keypoint1, index2, keypoint2)
int index_i, index_j;
Point2 p, measurement;
for (vector<tuple<int, Point2, int, Point2>>::iterator it = matches.begin();
    it != matches.end(); ++it) {

    std::tie(index_i, measurement, index_j, p) = *it;

    Point2_ predicted = transformTo(pose_expr, p);

    // Add the Point2 expression variable, an initial estimate, and the measurement noise.
    // The graph takes in factors with the robust error model.
    // This is the only change we have in our code example.
    graph.addExpressionFactor(predicted, measurement, huber);
}

// Optimize and print basic result
Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
result.print("Final Result:\n");
```

This version of the code gives us the `SE(2)` values `(4.75419, 3.60199, 0.139674)` which is pretty accurate, despite the initial estimate for the transform being arbitrary. This makes it apparent that the use of robust estimators and subsequently robust error models is the way to go for use cases where outliers are a concern. Of course, providing better initial estimates will only improve the final estimate.

But you may ask how does this compare to our dear old friend RANSAC? Surely RANSAC can perform as well as using the robust error model, so why do we even need all this extra overhead? Using the OpenCV implementation of RANSAC, a similar pipeline gives use the following `SE(2)` values: `(4.77360, 3.69461, 0.13960)`.

That's pretty close too, so why go through the headache of using robust error models? For one, unlike RANSAC, robust error models are deterministic and possess defined behavior. Moreover, one does not need to run multiple runs of optimization to obtain a consistent result, compared to RANSAC which may require hundreds of runs to converge to a good result.


<!-- ## Robust Error Models + RANSAC -->

## Conclusion

In this post, we have seen the basics of parameter estimation, a ubiquitous mathematical framework for many perception and robotics problems, and we have seen how this framework is susceptible to perturbations from outliers which can throw off the final estimate. More importantly, we have seen how a simple tool called the Robust M-estimator can easily help us deal with these outliers and their effects. An example case for `SE(2)` transform estimation demonstrates not only their ease of use with GTSAM, but also the efficacy of the solution generated, especially when compared to widely used alternatives such as RANSAC. 

Furthermore, robust estimators are deterministic, ameliorating the need for performing a large number of random restarts as is the case for RANSAC, which means our solution is also more computationally efficient. With the benefits of speed, accuracy, and ease of use, robust error models make a strong case for their adoption for many related problems and we hope you will give them a shot the next time you use GTSAM.

[1]: https://www.microsoft.com/en-us/research/wp-content/uploads/2016/11/RR-2676.pdf