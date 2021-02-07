---
layout: gtsam-post
title:  "Reducing the uncertainty about the uncertainties"
---

<link rel="stylesheet" href="/assets/css/slideshow.css">

Author: [Matias Mattamala](https://mmattamala.github.io)

<div style="display:none"> <!-- custom latex commands here -->
  $
    \usepackage{amsmath}
    \usepackage[makeroom]{cancel}
    \DeclareMathOperator*{\argmin}{argmin}
    \newcommand{\coloneqq}{\mathrel{:=}}
  $
</div>

<style>
.MJXc-display {
    overflow-x: auto;
    overflow-y: hidden;
    scrollbar-width: none; /* Firefox */
    -ms-overflow-style: none;  /* Internet Explorer 10+ */
}
.MJXc-display::-webkit-scrollbar {
    width: 5px;
    height: 2px;
}
.MJXc-display::-webkit-scrollbar-track {
    background: transparent;
}
.MJXc-display::-webkit-scrollbar-thumb {
    background: #ddd;
    visibility:hidden;
}
.MJXc-display:hover::-webkit-scrollbar-thumb {
    visibility:visible;
}
</style> <!-- horizontal scrolling -->

<!-- - TOC   -->
{:toc}

## Introduction
In this post I will review some general aspects of optimization-based state estimation methods, and how to input and output consistent quantities and uncertainties, i.e, covariance matrices, from them. We will take a (hopefully) comprehensive tour that will cover *why we do the things the way we do*, aiming to clarify some *uncertain* things about working with covariances. We will see how most of the explanations naturally arise by making explicit the definitions and conventions that sometimes we implicitly assume when using these tools.

This post summarizes and extends some of the interesting discussions we had in the [gtsam-users](https://groups.google.com/g/gtsam-users/c/c-BhH8mfqbo/m/7Wsj_nogBAAJ) group. We hope that such space will continue to bring to the table relevant questions shared by all of us.

## A simple example: a pose graph
As a motivation, we will use a similar pose graph to those used in other GTSAM examples:

<a name="fg_example"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/motivation.png"
    alt="Simple pose graph example" />
    <figcaption><b>Figure 1</b> We consider a robot moving on the 2D plane (top), which has an odometer that provides relative measurements of the robot's displacement. The graph at the bottom represent the factor graph model. Modeling the odometry factor in a consistent way is the main topic we will cover in this post.</figcaption>
</figure>
<br />

To start, we will consider that the variables in the graph $$\mathbf{x}_i$$
correspond to positions $${\mathbf{x}_{i}} = (x,y) \in \mathbb{R}^2$$. The variables are related by a transition matrix $$\mathbf{A}$$, as well as relative *measurements* $$\mathbf{b}_{i} = (b_x, b_y)$$ obtained from some sensor such as a wheel odometer. We can then establish the following relationships between variables $$i$$ and $$i+1$$:

$$
\begin{equation}
\mathbf{x}_{i+1} = \mathbf{A} \mathbf{x}_i + \mathbf{b}_i
\end{equation}
$$

However, we now that in reality things do not work that way, and we will usually have errors produced by noise in our sensors. The most common way to address this problem is adding some *zero-mean Gaussian noise* $$\eta_i\sim Gaussian(\mathbf{0}_{2\times1}, \Sigma_i)$$ to our measurement to model this uncertainty:

$$
\begin{equation}
\mathbf{x}_{i+1} = \mathbf{A}\mathbf{x}_i + \delta\mathbf{x}_i + \eta_i
\end{equation}
$$

We can recognize here the typical *motion* or *process model* we use in Kalman filter for instance, that describe how our state evolves. We say that the noise we introduced on the right side states that our next state $$\mathbf{x}_{i+1}$$ will be *around* $$\mathbf{A}\mathbf{x}_i + \mathbf{b}_i$$, and the covariance matrix $$\Sigma_i$$ describes the region where we expect $$\mathbf{x}_{i+1}$$ to lie.

We can also notice that with a bit of manipulation, it is possible to establish the following relationship:

$$
\begin{equation}
\eta_i  = \mathbf{x}_{i+1} - \mathbf{A}\mathbf{x}_i - \mathbf{b}_i
\end{equation}
$$

This is an important expression because we know that the left-hand expression distributes as a Gaussian distribution. But since we have an equivalence, the right-hand term must do as well. It is important to note here that what distributes as a Gaussian is neither $$\mathbf{x}_{i}$$ nor $$\mathbf{x}_{i+1}$$, but the difference $$(\mathbf{x}_{i+1} - \mathbf{A}\mathbf{x}_i + \mathbf{b}_i)$$. This allows us to use the difference as an  **odometry factor** that relates $$\mathbf{x}_i$$ and $$\mathbf{x}_{i+1}$$ probabillistically in our factor graph.

### Analyzing the solution
Solving the factor graph using the previous expression for the odometry factors is equivalent to solving the following least squares problem under the assumption that all our factors are Gaussian (which is fortunately our case):

$$
\begin{equation}
\mathcal{X}^{*} = \argmin\displaystyle\sum_{i} || {\mathbf{x}_{i+1}} - \mathbf{A}\mathbf{x}_{i} + \mathbf{b}_{i} ||^{2}_{\Sigma_i}
\end{equation}
$$

This problem is linear, hence solvable in closed form. By differentiating the squared cost and setting it to zero, we end up with the so-called *normal equations*, which are particularly relevant for our posterior analysis:

$$
\begin{equation}
\mathbf{A}^{T} \Sigma^{-1} \mathbf{A}\ \mathcal{X}^{*} = \mathbf{A}^{T} \Sigma^{-1} \mathbf{b}
\end{equation}
$$

First point we can notice here is that finding the solution $\mathcal{X}^{*}$ requires to invert the matrix $\mathbf{A}^{T} \Sigma^{-1} \mathbf{A}$, which in general is hard since it can be huge and dense in some parts. However we know that there are clever ways to solve it, such as iSAM and iSAM2 that GTSAM already implements, and are covered in [this comprehensive article](https://www.cc.gatech.edu/~dellaert/pubs/Dellaert17fnt.pdf) by Frank Dellart and Michael Kaess.

<a name="hessian"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/hessian.png"
    alt="Hessian matrix" />
    <figcaption><b>Figure 2</b> When solving the normal equations, the left-hand side is known as the Fisher information matrix or Hessian. Its inverse approximates the covariance of the least squares solution.</figcaption>
</figure>
<br />

Our interest in this matrix, though, known as **Fisher information matrix** or **Hessian** (since it approximates the Hessian of the original quadratic cost), is that its inverse $$\Sigma^{*} = (\mathbf{A}^{T} \Sigma^{-1} \mathbf{A})^{-1}$$ also approximates the covariance of our solution - known as *Laplace approximation* in machine learning ([Bishop (2006), Chap. 4.4](https://www.microsoft.com/en-us/research/uploads/prod/2006/01/Bishop-Pattern-Recognition-and-Machine-Learning-2006.pdf)). This is a quite important result because by solving the factor graph we are not only recovering an estimate of the mean of the solution, but also a measure of its uncertainty. In GTSAM, the `Marginals` class implements this procedure, and a example can be further explored in the [tutorial](https://gtsam.org/tutorials/intro.html#subsec_Full_Posterior_Inference).

As a result, we can say that after solving the factor graph the *probability distribution of the solution* is given by $$Gaussian(\mathcal{X}^{*}, \Sigma^{*})$$.


## Getting nonlinear
The previous pose graph was quite simple and probably not applicable for most of our problems. Having linear factors as the previous one is an *impossible dream* for most of the applications. It is more realistic to think that our state $$i+1$$ will evolve as a nonlinear function of the state $i$ and the measurements $$b_i$$:

$$
\begin{equation}
\mathbf{x}_{i+1} = f(\mathbf{x}_i, \mathbf{b}_i)
\end{equation}
$$

Despite nonlinear, we can still say that our mapping has some errors involved, which are embeded into a zero-mean Gaussian noise that we can simply add as before:

$$
\begin{equation}
\mathbf{x}_{i+1} = f(\mathbf{x}_i, \mathbf{b}_i) + \eta_i
\end{equation}
$$

So a similar expression follows by isolating the noise:

$$
\begin{equation}
\eta_i = \mathbf{x}_{i+1} - f(\mathbf{x}_i, \mathbf{b}_i)
\end{equation}
$$

Now the factors are defined by the residual $$\mathbf{x}_{i+1} - f(\mathbf{x}_i, \mathbf{b}_i)$$, which poses the following *nonlinear least squares* (NLS) problem:

$$
\begin{equation}
\mathcal{X} = \argmin \displaystyle\sum_i || \mathbf{x}_{i+1} - f(\mathbf{x}_i, \mathbf{b}_i) ||^2_{\Sigma_i}
\end{equation}
$$

Since the system is not linear anymore we cannot solve it in close form. We need to use nonlinear optimization algorithms such as Gauss-Newton or Levenberg-Marquardt. They will try to approximate our *linear dream* by linearizing the residual with respect to some **linearization or operation point** given by a guess $$\mathcal{\bar{X}}^{k}$$ valid at iteration $k$:

$$
\begin{equation}
f(\mathbf{x}_i, \mathbf{b}_i) \approx f(\mathbf{\bar{x}}^{k}, \mathbf{b}_i) + \mathbf{H}(\bar{x}^{k})\mathbf{x}_i = \mathbf{b}_k + \mathbf{H}^k \mathbf{x}_i
\end{equation}
$$

Hence the problem becomes:

$$
\begin{equation}
\delta\mathcal{X}^{k}= \argmin \displaystyle\sum_i || \mathbf{x}_{i+1} - \mathbf{H}^k\mathbf{x}_i - \mathbf{b}_i ||^2_{\Sigma_i^k}
\end{equation}
$$

It is important to observe here that we are not obtaining the global solution $\mathcal{X}$, but just a small increment $\delta\mathcal{X}$ that will allows to move closer to some minima that depends on the initial values. This linear problem **can** be solved in closed form as we did before. In fact, the normal equations now become:

$$
\begin{equation}
\label{nonlinear-normal-equations}
(\mathbf{H}^k)^{T} (\Sigma^{k})^{-1}\ \mathbf{H}^k\ \delta\mathcal{X} = (\mathbf{H}^k)^{T} (\Sigma^{k})^{-1} \mathbf{b}
\end{equation}
$$

The solution $\delta\mathcal{X}$ will be used to update our current solution at iteration $k$ using the update rule:

$$
\begin{equation}
\mathcal{X}^{k+1} = \mathcal{X}^k + \delta\mathcal{X}^{k}
\end{equation}
$$

where $$\mathcal{X}^{k+1}$$ corresponds to the best estimate so far, and can be used as a new linearization point for a next iteration.

Similarly, the expression on the left-hand side $\Sigma^{k+1} = (\mathbf{A}^{T} (\Sigma^{k})^{-1} \mathbf{A})^{-1}$ also corresponds to the Fisher information or Hessian as before, **but with respect to the linearization point**. This is quite important because both the best solution so far $\mathcal{X}^{k+1}$ and its associated covariance $\Sigma^{k+1}$ will be valid only at this linearization point and will change with every iteration of the nonlinear optimization. But understanding this, we still can say that **at iteration $k+1$ our best solution will follow a distribution** $Gaussian(\mathcal{X}^{k+1}, \Sigma^{k+1})$.


## Getting non-Euclidean
So far so good, but we need to admit we were not too honest before when we said that considering nonlinear functions of the variables was everything we needed to model real problems. The previous formulation assumed that the variables we aim to estimate are **vectors**, which is not the case for robotics and computer vision at least.

In robotics, we do not estimate the state of a robot only by their position but also its orientation. Then we say that is its **pose**, i.e position and orientation together, what matters to define its state.

Representing pose is a tricky thing. We could say *"ok, but let's just append the orientation to the position vector and do everything as before"* but that does not work in practice. Problem arises when we want to compose two poses $\mathbf{T}_1 = (x_1, y_1, \theta_1)$ and $\mathbf{T}_2 = (x_2, y_2, \theta_2)$. Under the vector assumption, we can write the following expression as the composition of two poses:

$$
\begin{equation}
\mathbf{T}_1 + \mathbf{T}_2 = \begin{bmatrix} x_1\\ y_1 \\ \theta_1 \end{bmatrix}  +  \begin{bmatrix} x_2 \\ y_2 \\ \theta_2\end{bmatrix}  = \begin{bmatrix} x_1 + x_2 \\ y_1+y_2 \\ \theta_1 + \theta_2\end{bmatrix}
\end{equation}
$$

This is basically saying that _it does not matter if we start in pose $\mathbf{T}_1$ or $\mathbf{T}_2$, we will end up at the same final pose_ by composing both, because in vector spaces we can commute the elements. **But this does not work in reality, because rotations and translations do not commute**. A simple example is that if we start at a certain position, walk 1 step forward and then turn right, we will end up in a different place that if we turn right first and then walk the step forward.

So we need a different representation for poses that allow us to describe accurately what we observe in reality. Long story short, we rather prefer to represent poses as $3\times3$ matrices known as *rigid-body transformations*:

$$
\begin{equation}
\mathbf{T}_1 = \left[\begin{matrix} \cos{\theta_1} && -\sin{\theta_1} && x_1 \\ \sin{\theta_1} && \cos{\theta_1} && y_1 \\ 0 && 0 && 1\end{matrix} \right] = \left[\begin{matrix} \mathbf{R}_1 && \mathbf{t}_1 \\ 0 && 1\end{matrix}\right]
\end{equation}
$$

Here $\mathbf{R}_1 = \left[ \begin{matrix} \cos{\theta_1} && -\sin{\theta_1}\\ \sin{\theta_1} && \cos{\theta_1}\end{matrix} \right]$ is a 2D rotation matrix, while $\mathbf{t}_1 = \left[ \begin{matrix}x_1 \\ y_1 \end{matrix}\right]$ is a translation vector.
While we are using a $3\times3$ matrix now to represent the pose, *its degrees of freedom* are still $3$, since it is a function of $(x_1, y_1, \theta_1)$.

Working with transformation matrices is great, because we can now describe the behavior we previously explained with words now using matrix operations. If we start in pose $\mathbf{T}_1$ and we apply the transformation $\mathbf{T}_2$:

$$
\begin{equation}
\mathbf{T}_1 \mathbf{T}_2 = \left[\begin{matrix} \mathbf{R}_1 && \mathbf{t}_1 \\ 0 && 1\end{matrix}\right] \left[\begin{matrix} \mathbf{R}_2 && \mathbf{t}_2 \\ 0 && 1\end{matrix}\right] = \left[\begin{matrix} \mathbf{R}_1 \mathbf{R}_2 && \mathbf{R}_1 \mathbf{t}_2 +  \mathbf{t}_1 \\ 0 && 1\end{matrix}\right]
\end{equation}
$$

We can now rewrite the same problem as before but now using our transformation matrices:

\begin{equation}
\mathbf{T}_{i+1} = \mathbf{T}_i \ \Delta\mathbf{T}_i
\end{equation}

Here we are saying that the next state $$\mathbf{T}_{i+1}$$ is gonna be the previous state $$\mathbf{T}_{i}$$ *plus* some increment $$\Delta\mathbf{T}_i$$ given by a sensor -odometry in this case. Please note that since we are now working with transformation matrices, $$\Delta\mathbf{T}_i$$ is almost everything we need to model the problem more accurately, since it will represent a change in both position and orientation.

Now, **there are two things here that we must discuss** before moving on, which are fundamental to make everything consistent. We will review them carefully now.

### The importance of coordinate frames
While some people (myself included) prefer to write the process as we did before, applying the increment on the **right-hand** side:

$$
\begin{equation}
\mathbf{T}_{i+1} = \mathbf{T}_i \ \Delta\mathbf{T}_i
\end{equation}
$$

Others do it in a different way, on the **left-hand** side:

$$
\begin{equation}
\mathbf{T}_{i+1} = \Delta\mathbf{T}_i \ \mathbf{T}_i
\end{equation}
$$

These expressions are not equivalent as we already discussed because rigid-body transformations do not commute. However, **both make sense under specific conventions**. We will be more explicit now by introducing the concept of **reference frames** in our notation. We will use the notation presented by Paul Furgale in his blog post ["Representing Robot Pose: The good, the bad, and the ugly"](http://paulfurgale.info/news/2014/6/9/representing-robot-pose-the-good-the-bad-and-the-ugly).

Let us say that the robot trajectory in space is expressed in a fixed frame; we will called it the *world frame* $W$. The robot itself defines a coordinate frame in its body, the *body frame* $B$.

Then we can define the **pose of the robot body $B$ expressed in the world frame $W$** by using the following notation:

$$
\begin{equation}
\mathbf{T}_{WB}
\end{equation}
$$

Analogously, using we can express **the pose of the world $W$ expressed in the robot frame $B$**:

$$
\begin{equation}
\mathbf{T}_{BW}
\end{equation}
$$

According to this formulation, the first is a description *from a fixed, world frame* while the left-hand one does it *from the robot's perspective*. The interesting thing is that we can always go from one to the other by inverting the matrices:

$$
\begin{equation}
\mathbf{T}_{WB}^{-1} = \mathbf{T}_{BW}
\end{equation}
$$

So the inverse *swaps* the subindices, effectively changing our *point-of-view* to describe the world.

**Using one or the other does not matter. The important thing is to stay consistent**. 
In fact, by making the frames explicit in our convention we can always check if we are making mistakes. Let us say we are using the first definition, and our odometer is providing increments _relative to the previous body state_. Our increments $$\Delta\mathbf{T}_i$$ will be $$\Delta\mathbf{T}_{B_{i} B_{i+1} }$$, i.e, it is a transformation that describes the pose of the body at time $i+1$ expressed in the previous one $i$.

Hence, the actual pose of the body at time $i+1$ expressed in the world frame $W$ is given by **matrix multiplication of the increment on the right side**:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1} }
\end{equation}
$$

We know this is correct because the inner indices cancel each other, effectively representing the pose at $i+1$ in the frame we want:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{W {\cancel{B_i}} } \ \Delta\mathbf{T}_{ {\cancel{B_{i}}} B_{i+1} }
\end{equation}
$$

Since we applied the increment from the right-hand side, we will refer to this as the **right-hand convention**.

<a name="right-convention"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/right-convention.png"
    alt="Hessian matrix" />
    <figcaption>When using right-hand convention, the increments to describe the pose of the robot at instant $i$ are applied on the right-hand side. All our estimates are expressed with respect to a fixed world frame $W$ (in black).</figcaption>
</figure>
<br />

If we want to apply the increment to $\mathbf{T}_{BW}$, we would need to invert the measurement:

$$
\begin{equation}
\Delta\mathbf{T}_{B_{i+1} B_{i}} = \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}
\end{equation}
$$

so we can apply the transformation on the left-hand side:

$$
\begin{equation}
\mathbf{T}_{B_{i+1}W} = \Delta\mathbf{T}_{B_{i+1} B_{i}} \mathbf{T}_{B_i W}
\end{equation}
$$

which we will refer onward as **left-hand convention**.

<a name="left-convention"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/left-convention.png"
    alt="Hessian matrix" />
    <figcaption>With left-hand convention the increments are applied from the left, changing our reference frame (in black) to the new robot pose $B_{i+1}$.</figcaption>
</figure>
<br />


**In GTSAM we use the right-hand convention: we assume that the variables are expressed with respect to a fixed frame $W$, hence the increments are applied on the right-hand side**. 

It is important to have this in mind because all the operations that are implemented follow this. As a matter of fact, 3D computer vision has generally used left-hand convention because it is straightforward to apply the projection models from the left:

$$
\begin{equation}
{_I}\mathbf{p} = {_{I}}\mathbf{K}_{IC}\ \mathbf{T}_{CW}\ {_{W}}\mathbf{P}
\end{equation}
$$

Here we made explicit all the frames involved in the transformations we usually can find in textbooks: An homogeneous point $${_{W}}\mathbf{P}$$ expressed in the world frame $W$, is transformed by the *extrinsic calibration matrix* $$\mathbf{T}_{CW}$$ (a rigid body transformation ) which represents the world $W$ in the camera frame $C$, producing a vector $${_{C}}\mathbf{P}$$ in the camera frame. This is projected onto the image by means of the intrinsic calibration matrix $${_{I}}\mathbf{K}_{IC}$$, producing the vector $$_I\mathbf{p}$$, expressed in the image frame $I$.

Since in GTSAM the extrinsic calibration is defined the other way, i.e $\mathbf{T}_{WC}$, the implementation of the `CalibratedCamera` handles it by [properly inverting the matrix before doing the projection](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/CalibratedCamera.cpp#L120) as we would expect.

### They are not just matrices, they are _manifolds_

The second important point we need to discuss, is that while rigid-body transformations are nice, the new expression we have to represent the process presents some challenges:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1} }
\end{equation}
$$

In first place, we need to figure out a way to include the *noise* term $$\eta_i$$ we used before to handle the uncertainties about our measurement $$\Delta\mathbf{T}_{B_{i} B_{i+1} }$$, which was also the *trick* we used to generate the Gaussian factors we use in our factor graph. For now, we will say that the noise will be given by a matrix $$\mathbf{N}_i \in \mathbb{R}^{4\times4}$$, which holds the following:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}} \mathbf{N}_i
\end{equation}
$$

Secondly, assuming the noise $$\mathbf{N}_i$$ was defined somehow, we can isolate it as we did before. However, we need to use matrix operations now:

$$
\begin{equation}
\mathbf{N}_i = (\mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}})^{-1} \mathbf{T}_{WB_{i+1}}
\end{equation}
$$

and we can manipulate it a bit for clarity:

$$
\begin{equation}
\mathbf{N}_i = (\Delta\mathbf{T}_{B_{i} B_{i+1}})^{-1} (\mathbf{T}_{WB_i})^{-1} \mathbf{T}_{WB_{i+1}}
\end{equation}
$$

If we do the subindex cancelation trick we did before, properly swapping the indices due to the inverses, we can confirm that the error is defined *in frame* $$B_{i+1}$$ *and expressed in frame* $B_{i+1}$. This may seen counterintuitive, but in fact describes what we want: the degree of mismatch between our models and measurements to express the pose at frame $$B_{i+1}$$, which ideally should be zero.

However, the error is still a matrix, which is impossible to include as a factor in the framework we have built so far. We cannot compute a *vector residual* as we did before, nor we are completely sure that the matrix $$\mathbf{N}_i$$ follows some sort of Gaussian distribution.

#### Manifolds
Here is where the concept of **manifold** comes to solve our problems. Rigid-body transformations (`Pose3` and `Pose2` in GTSAM), rotation matrices (`Rot2` and `Rot3`), quaternions and even vector spaces (`Point2` and `Point3`) are **differentiable manifolds**. This means that in spite of they do not behave as Euclidean spaces at a global scale, they can be *locally approximated* as such by using local vector spaces called **tangent spaces**. The main advantage of analyzing all these objects from the manifold perspective is that we can build general algorithms based on common principles that apply to all of them.


<a name="manifold"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/manifold.png"
    alt="Manifold and tangents spaces" />
    <figcaption> While a manifold have a non-Euclidean structure, it can be locally approximated by tangent spaces. </figcaption>
</figure>
<br />

In order to work with all the previous objects we mentioned -rotation matrices, rigid-body transformations, quaternions-, we need to define some basic operations that resemble the vector case that allows us to generalize the framework. These concepts [have been discussed in previous posts](https://gtsam.org/notes/GTSAM-Concepts.html) at the implementation level:

1. **Composition**: How to compose 2 objects from the same manifold, with associativity properties. It is similar to the *addition* operation for vectors.
2. **Between**: Computes the difference between 2 objects from the same manifold. Similar to a *subtraction* operation.
3. **Identity**: An identity operator under the composition/between.
4. **Inverse**: An element that when composed with its inverse becomes the identity.
5. **Local**: An operation that maps elements from the manifold to the tangent space.
6. **Retract or retraction**: The opposite operation: mapping from the tangent space back to the manifold.

The first 4 are basic properties we need to define a [**group**](https://en.wikipedia.org/wiki/Group_(mathematics)). In fact, the *between* operation is simply a by-product of having the composition, inverse and identity well-defined. We can also notice that they are operations we already used before in this post when working with rigid-body matrices. The only difference is that now we defined them in a more general way.

Some authors define special operators to refer to composition/between operations, such as *box-plus* $\boxplus$ for *composition* and *box-minus* $\boxminus$ for *between* as done by [Hertzberg et al.](https://arxiv.org/abs/1107.1119) for general manifolds, and [Bloesch et al.](https://arxiv.org/abs/1606.05285) and the [Kindr library](https://github.com/ANYbotics/kindr) for rotations. However, composition can be still defined from the *left* or the *right* side because **composition has associativity but is not distributive**. This is **exactly the problem we described before when talking about reference frames and how to add small increments with the _left_ and _right_ hand conventions, since both are valid depending on our decisions or other authors.** While we can be clear about our own decisions, it is important to be aware of the definitions of each author because sometimes are not clearly stated. [Solà et al](https://arxiv.org/abs/1812.01537) for example make the difference explicit by defining *left-*$\oplus$ and *right-*$\oplus$ for composition using *left-hand convention* or *right-hand convention* respectively, but we need to be careful to recognize each other's choices.

**We recall again that in GTSAM and the rest of this post we use the _right_ convention** (*pun intended*), because we represent our quantities with respect to a fixed world frame $W$ and the increments are defined with respect to the base frame $B$.

#### Local and retract
Regarding the last 2 properties, **local** and **retract** operations are the key to work with manifolds. As we briefly mentioned before, objects such as rotation matrices and rigid-body transformations are difficult to manipulate in the estimation framework because they are matrices. A 3D rotation matrix $\mathbf{R}$ represents 3 orientations with respect to a reference frame but, in raw terms, they are using 9 values to do so, which seems to *overparametrize* the object. However, the constraints that define a rotation matrix -and consequently the manifold- such as orthonormality $$\mathbf{R}^{T}\mathbf{R} = \mathbf{I}$$ and $$\text{det}(\mathbf{R}) = 1$$ make the inherent dimensionality of the rotation still 3. Interestingly, this is exactly the dimensionality of the tangent spaces that can be defined over the manifold.


<a name="manifold_local"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/manifold-local.png"
    alt="Local operation on a manifold" />
    <figcaption>The local operation allows us to map elements from the manifold to the tangent space.</figcaption>
</figure>
<br />

<a name="manifold_retract"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/manifold-retract.png"
    alt="Retract operation on a manifold" />
    <figcaption>The retract operation does the opposite mapping vectors from the tangent space back on the manifold.</figcaption>
</figure>
<br />

**That is what makes working with manifolds so convenient**: All the constraints that are part of the definition of the object are naturally handled, and we can work in local (tangent) vector spaces using their *inherent* dimension. The same happens for rigid-body transformations (6 dimensions represented by a 16 elements matrix) or quaternions (3 orientations represented by a 4D vector).

The mapping **from the manifold to the tangent space** is given by the **local** operator. Conversely, we can go **from the tangent space to the manifold** by using the **retract** (or retraction) one.

People familiar with Lie groups will ask what are the differences with them: Lie groups are defined as groups that are also differentiable manifolds. Indeed, rigid-body transformations are objects of the *Special Euclidean group* $\text{SE(3)}$ and we use those definitions to define the operations:

1. **Composition**: Matrix multiplication $$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1} }$$.
2. **Between**: $$\Delta\mathbf{T}_{B_{i} B_{i+1}} = (\mathbf{T}_{WB_i})^{-1} \  \mathbf{T}_{WB_{i+1}}$$.
3. **Identity**: Identity matrix $$\mathbf{I}_W$$.
4. **Inverse**: Matrix inverse $$(\mathbf{T}_{WB_i})^{-1} = \mathbf{T}_{B_i W}$$
5. **Local**: We use the _logarithm map_ of $$\text{SE(3)}$$: $$_W\mathbf{\xi}_{W} = \text{Log}(\mathbf{T}_{WB_i} )$$.
6. **Retract**: Analogously, we use the _exponential map_ of $$\text{SE(3)}$: $\mathbf{T}_{WB_i} = \text{Exp}(_W\mathbf{\xi}_{W})$$.

(Please note here that we used *capitalized* $$\text{Log}(\cdot) := \text{log}( \cdot)^{\vee}$$ and $$\text{Exp}(\cdot):=\text{exp}( (\cdot)^{\wedge})$$ operators for simplicity as used by   [Forster et al (2017),](https://arxiv.org/abs/1512.02363) and [Solà et al. (2020)](https://arxiv.org/abs/1812.01537).)

The main difference with Lie groups is that by using the general concept of [_retraction_](https://press.princeton.edu/absil) we can use **alternative definitions to go from the tangent space to the manifold, that can be more efficient than the exponential map when solving optimization problems.**

It is also important to notice that **the reference frames are preserved when applying the local and retract operations**. For instance, when using the *local* operation we defined using the logarithm map of  $$\text{SE(3)}$$, we obtain a vector $$_W\mathbf{\xi}_{W} \in \mathbb{R}^{6}$$ (sometimes also called *tangent vector* or *twist*), which is defined in the tangent space *centered at the world frame* in this case:

$$
\begin{equation}
{_W}\mathbf{\xi}_{W} = \text{Log}(\mathbf{T}_{WB_i} )
\end{equation}
$$

The same property holds to *add incremental changes to a transformation*:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \text{Exp}( {_{B_i}}\mathbf{\xi}_{B_i})
\end{equation}
$$

In this case we added an increment from the base frame at time $i$, that represents the new pose at time $i+1$. Please note that the increments are defined with respect to a reference frame, but they do not require to specify the resulting frame. Their meaning (representing a new pose at time $i+1$) is something that we -as users- define but is not explicit in the formulation. (*While we could do it, it can lead to confusions because in this specific case we are representing the pose at the next instant but we can also use retractions to describe corrections to the base frame as we will see later.*)

 The incremental formulation via retractions is particularly convenient when we have local (base frame) velocity measurements $$({_B}\omega_B, {_B}{v_B})$$, with $${_B}\omega_B \in \mathbb{R}^{3}, {_B}v_B \in \mathbb{R}^{3}$$  and we want to do [*dead reckoning*](https://en.wikipedia.org/wiki/Dead_reckoning):

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = {\mathbf{T}_{WB_i}} \text{Exp}\left(
  \begin{bmatrix} _B\omega_B \\ _B v_B \end{bmatrix}\ 
\delta t \right)
\end{equation}
$$

The product
$$\begin{bmatrix} _B\omega_B \ \delta t \\ _B v_B \ \delta t\end{bmatrix}$$
represents the tangent vector resulting from time-integrating the velocity, which is map onto the manifold by means of the $\text{SE(3)}$ retraction. 

**We need to be careful about the convention of the retraction/local operation** (yes, more conventions again). Having clarity about the definition that every software defines for these operations (even implicitly) is fundamental to make sense of the quantities we put into our estimation problems and the estimates we extract. For instance, the definition of the $\text{SE(3)}$ retraction we presented, which matches `Pose3` in GTSAM, uses an _orientation-then-translation_ convention, i.e, the 6D tangent vector has orientation in the first 3 coordinates, and translation in the last 3. On the other hand, `Pose2` uses _translation-then-orientation_ $(x, y, \theta)$ for [historical reasons](https://github.com/borglab/gtsam/issues/160#issuecomment-562161665).

#### Adjoints
Lastly, we need to discuss an operator that will be useful to operate covariances correctly (spoiler) when we return to our original estimation problem. To introduce it, let us consider the example before in which we added an incremental change in the base frame:

$$
\begin{equation}
\mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i})
\end{equation}
$$

While it could not be straightforward, _it is possible_ to found an incremental change applied at the world frame $$_{W}\mathbf{\xi}_{W}$$ that can lead to the same result:

$$
\begin{equation}
\text{Exp}( _{W}\mathbf{\xi}_{W}) \mathbf{T}_{WB_i} = \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i})
\end{equation}
$$

In order to satisfy this condition, the incremental change $$_{W}\mathbf{\xi}_{W}$$ must be given by:

$$
\begin{equation}
\text{Exp}( _{W}\mathbf{\xi}_{W}) = \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i}) \mathbf{T}_{WB_i}^{-1}
\end{equation}
$$

which we obtained by isolating the term. The expression on the right-hand side is known as the _adjoint action_ of $\text{SE(3)}$. This relates increments applied on the left to ones applied on the right. For our purposes, it is useful to use an equivalent alternative expression that applies directly to elements from the tangent space ([Solà et al.](https://arxiv.org/abs/1812.01537) gives a more complete derivation as a result of some properties we omitted here):

$$
\begin{equation}
\text{Exp}( _{W}\mathbf{\xi}_{W}) \mathbf{T}_{WB_i} = \mathbf{T}_{WB_i} \text{Exp}( \text{Ad}_{T_{WB_i}^{-1}}  {_{W}}\mathbf{\xi}_{W})
\end{equation}
$$

where $$\text{Ad}_{T_{WB_i}^{-1}}$$ is known as the _adjoint matrix_ or **_adjoint_ of $$T_{WB_i}^{-1}$$**. The adjoint acts over elements of the tangent space directly, changing their reference frame. Please note that the same subindex cancelation applies here, so we can confirm that the transformations are correctly defined.

We can also interpret this as a way to _move_ increments applied on the left-hand side to the right-hand side, which is particularly useful to keep the right-hand convention used in GTSAM consistent.

## Bringing everything together
Now we can return to our original estimation problem using rigid-body transformations. Let us recall that we defined the following process model given by odometry measurements:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}} \mathbf{N}_i
\end{equation}
$$

We reported two problems before:
1. We needed to define the noise $\mathbf{N}_i$ as a Gaussian noise, but it was a $6\times6$ matrix.
2. When isolating the noise to create the residual, we ended up with a matrix expression, not vector one as we needed in our estimation framework.

The first problem can be solved by using the tools we just defined by defining _a zero-mean Gaussian in the tangent space of $\text{SE(3)}$ and retracting it onto the manifold_:

$$
\begin{equation}
\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}} \text{Exp}( _{B_i}\mathbf{\eta}_{B_i})
\end{equation}
$$

where we have defined $$\eta_{B_i} \sim Gaussian(\mathbf{0}_{6\times1}, \Sigma_i)$$. Please note that in order to match our right-hand convention, **the covariance we use must be defined in the base frame**. Additionally, **the covariance matrix must follow the same ordering defined by the retraction**. For `Pose3` objects, for instance, the upper-left block must encode orientation covariances, while the bottom-right position covariances:

$$
\begin{equation}
\left[\begin{matrix} \Sigma_{\phi\phi} & \Sigma_{\phi\rho} \\ \Sigma_{\phi\rho}^{T} & \Sigma_{\rho\rho} \end{matrix}\right]
\end{equation}
$$

where we have used $\phi$ to denote orientation components while $\rho$ for the translational ones. 


As a side note, we can use the same procedure to define **Gaussian distributions for any transformation** $$\mathbf{T}_{WB}$$:

$$
\begin{equation}
\mathbf{\tilde{T}}_{WB} = \mathbf{T}_{WB} \text{Exp}( _{B}\mathbf{\eta}_{B})
\end{equation}
$$

This definition has been widely used in the past in estimation problems and matches the one used by GTSAM. In the literature, however, definitions differ in how the Gaussian is retracted (left-hand or right-hand), similarly to the differences observed with the composition operation. [Barfoot and Furgale (2014, left-hand convention)](http://ncfrn.cim.mcgill.ca/members/pubs/barfoot_tro14.pdf), [Forster et al (2017, right-hand convention)](https://arxiv.org/abs/1512.02363), and [Mangelson et al. (2020, left-hand convention)](https://arxiv.org/abs/1906.07795) are some examples. Other definitions to define probability distributions on manifolds include [Calinon (2020)](https://arxiv.org/abs/1909.05946) and [Lee et al. (2008)](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.175.5054&rep=rep1&type=pdf), please refer to their work for further details.

<a name="manifold_gaussian"></a>
<figure class="center">
  <img src="/assets/images/uncertainties/manifold-gaussian.png"
    alt="Gaussian on a manifold" />
    <figcaption>Using the retraction, we can define Gaussians on the tangent space and map them back on the manifold to construct Gaussians on the manifold.</figcaption>
</figure>
<br />

Having solved the first problem, we can now focus on the residual definition. We can isolate the noise as we did before, which holds:

$$
\begin{equation}
\text{Exp}( {_{B_{i+1}}}\mathbf{\eta}_{B_{i+1}}) = \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \ \mathbf{T}_{WB_{i+1}}
\end{equation}
$$

However, we can now apply the **local** operator of the manifold on both sides, which corresponds to the _logarithm map_ for $\text{SE(3)}$ elements:

$$
\begin{equation}
_{B_{i+1}}\mathbf{\eta}_{B_{i+1}} = \text{Log}\left( \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \  \mathbf{T}_{WB_{i+1}} \right)
\end{equation}
$$

Since the noise is defined in the tangent space, both sides denote vector expressions in $\mathbb{R}^{6}$. Both also correspond to zero-mean Gaussians, hence the right-hand side can be used as a proper factor in our estimation framework. In fact, the expression on the right side is _exactly_ the same used in GTSAM to define the [`BetweenFactor`](https://github.com/devbharat/gtsam/blob/master/gtsam/slam/BetweenFactor.h#L90).

We must also keep in mind here that by using the **local** operation, **the residual vector will follow the same ordering**. As we mentioned before, for `Pose3` objects, it will encode orientation error in the first 3 components, while translation error in the last ones. In this way, if we write the expanded expression for the Gaussian factor, we can notice that all the components are weighted accordingly (orientation first, and then translation):

$$
\begin{align}
\mathbf{r}_{odom}(\mathbf{T}_{WB_i}, \mathbf{T}_{WB_{i+1}}) &= \left|\left| \text{Log}\left( \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \  \mathbf{T}_{WB_{i+1}} \right)\right|\right|^{2}_{\Sigma_i} \\
& =
\text{Log}\left( \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \  \mathbf{T}_{WB_{i+1}} \right)^{T} \ \Sigma_i^{-1} \ \text{Log}\left( \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \  \mathbf{T}_{WB_{i+1}} \right)
\end{align}
$$

The factor is now a nonlinear vector expression that can be solved using the nonlinear optimization techniques we presented before. In fact, GTSAM already implements Jacobians for the **local** operator of all the objects implemented, which simplifies the process. However, there are subtle differences that we must clarify.

First, since the factor defines a residual in the tangent space at the current linearization point, the optimization itself is executed **in the tangent space defined in the current linearization point_**. This means that when we linearize the factors and build the normal equations, the increment ${_{B_i}}\delta\mathbf{T}^{k}$ we compute lies in the tangent space.

For this reason, we need to update the variables _on the manifold_ using the retraction:

$$
\begin{equation}
\mathbf{\tilde{T}}_{WB_i}^{k+1} = \mathbf{T}_{WB_i}^k \text{Exp}( {_{B_i}}\delta\mathbf{T}^{k} )
\end{equation}
$$

The second important point, is that the covariance that we can recover from the information matrix, **will be also defined in the tangent space around the linearization point, following the convention of the retraction**. It means that if our information matrix at the current linearization point is given by:

$$
\begin{equation}
\Sigma^{k+1} = (\mathbf{A}^{T} (\Sigma^{k})^{-1} \mathbf{A})^{-1}
\end{equation}
$$

Then, the corresponding distribution of the solution around the current linearization point $\mathbf{T}_{WB_i}^{k+1}$ will be given by:

$$
\begin{equation}
Gaussian(\mathbf{T}_{WB_i}^{k+1}, \Sigma^{k+1}) = \mathbf{T}_{WB_i}^{k+1} \text{Exp}( {_{B_i}}\eta_{B_i}^{k+1} )
\end{equation}
$$

where $${_{B_i}}\eta_{B_i}^{k+1} \sim Gaussian(\mathbf{0}_{6\times1}, \Sigma^{k+1})$$. As a consequence of the convention on the retraction, **the resulting covariance is expressed in the base frame as well, and uses orientation-then-translation for the ordering of the covariance matrix**.

## Playing with covariances
In this last section, we would like to discuss some consequences of the definitions, and how they can be used to obtain other useful expressions for covariance transformations. We will focus on $\text{SE(3)}$, but similar definitions should apply for other manifolds since they mainly rely on a definition of the adjoint.

Most of this expressions have been already shown in the literature by [Barfoot and Furgale (2014)](http://ncfrn.cim.mcgill.ca/members/pubs/barfoot_tro14.pdf) and [Mangelson et al. (2020)](https://arxiv.org/abs/1906.07795) but since they follow the left-hand convention they are not straightforward to use with GTSAM. We provide the resulting expressions for the covariance transformations following Mangelson et al. but we recommend to refer to their work to understand the details of the process.
### Distribution of the inverse
Understanding how the covariances get transformed when we invert a rigid-body matrix is useful to express the covariances in a different frame. In particular, if we want to analyze how they evolve using a fixed frame for instance. Let us consider we have the distribution:

$$
\begin{equation}
\mathbf{\tilde{T}}_{WB} = \mathbf{T}_{WB} \text{Exp}( _{B}\mathbf{\eta}_{B})
\end{equation}
$$

with $$_{B}\mathbf{\eta}_{B}$$ zero-mean Gaussian noise with covariance $\Sigma_{B}$ as before. The distribution of the inverse can be computed by inverting the expression:

$$
\begin{align}
(\mathbf{\tilde{T}}_{WB})^{-1} & = (\mathbf{T}_{WB} \text{Exp}( _{B}\mathbf{\eta}_{B}) )^{-1}\\
& = (\text{Exp}( _{B}\mathbf{\eta}_{B}) )^{-1}\ \mathbf{T}_{WB}^{-1}\\
& = \text{Exp}(- _{B}\mathbf{\eta}_{B}) \ \mathbf{T}_{WB}^{-1}
\end{align}
$$

However, the _noise_ is defined on the left, which is inconvenient to be consistent with right-hand convention. We can move it to the right using the adjoint:

$$
\begin{equation}
(\mathbf{\tilde{T}}_{WB})^{-1} = \ \mathbf{T}_{WB}^{-1}\ \text{Exp}(- \text{Ad}_{\mathbf{T}_{WB}} {_{B}}\mathbf{\eta}_{B})
\end{equation}
$$

This is a proper distribution following the right-hand convention. The covariance of the inverse is given by:

$$
\begin{equation}
\Sigma_{W} = \text{Ad}_{\mathbf{T}_{WB}} \Sigma_B \text{Ad}_{\mathbf{T}_{WB}}^{T}
\end{equation}
$$

### Distribution of the composition
A similar procedure can be followed to compute the covariance of the composition of poses. This is useful when doing dead reckoning for instance:

$$
\begin{align}
\mathbf{\tilde{T}}_{WB_i} &= \mathbf{\tilde{T}}_{WB_i} \mathbf{\tilde{T}}_{B_i B_{i+1}}\\
&= \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\eta}_{B_i})\ \mathbf{T}_{B_i B_{i+1}} \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})
\end{align}
$$

Analogously, we need to _move_ the noise $_{B_i}\mathbf{\eta}_{B_i}$ to the right, so as to have _the transformations to the left, and the noises to the right_. We can use the adjoint again:

$$
\begin{equation}
\mathbf{\tilde{T}}_{WB_i} = \mathbf{T}_{WB_i} \ \mathbf{T}_{B_i B_{i+1}} \text{Exp}(\text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} {_{B_i}}\mathbf{\eta}_{B_i})\  \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})
\end{equation}
$$

However, we cannot combine the exponentials because that would assume commutativity that does not hold for $\text{SE(3)}$ as we discussed previously. However, it is possible to use some approximations (also discussed in Mangelson's) to end up with the following expressions for the covariance of the composition:

$$
\begin{equation}
\Sigma_{B_{i+1}} = \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} \Sigma_{B_{i}} \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}}^{T} + \Sigma_{B_{i+1}}
\end{equation}
$$

Additionally, if we consider that the poses are correlated and then the covariance of the joint distribution is given by:

$$
\begin{equation}
\begin{bmatrix} \Sigma_{B_{i}} & \Sigma_{B_{i}, B_{i+1}} \\ \Sigma_{B_{i}, B_{i+1}}^{T} & \Sigma_{B_{i+1}} \end{bmatrix}
\end{equation}
$$

Then, the covariance of the composition is:

$$
\begin{equation}
\Sigma_{B_{i+1}} = \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} \Sigma_{B_{i}} \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}}^{T} + \Sigma_{B_{i+1}} +  \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} \Sigma_{B_{i}, B_{i+1}} +  \Sigma_{B_{i}, B_{i+1}}^{T} \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}}^{T}
\end{equation}
$$

### Distribution of the relative transformation
Computing the distribution of relative poses is relevant when we have an odometry system that provides estimates with covariance, and we want to use relative measurements for a pose graph SLAM system for instance. It follows the same procedure:

$$
\begin{align}
\mathbf{\tilde{T}}_{B_i B_{i+1}} &= \mathbf{\tilde{T}}_{W B_{i}}^{-1} \mathbf{\tilde{T}}_{WB_{i+1}}\\
 &= \left( \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\eta}_{B_i}) \right)^{-1} \ \mathbf{T}_{WB_{i+1}} \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})\\
 &= \text{Exp}(- _{B_i}\mathbf{\eta}_{B_i}) \ \mathbf{T}_{WB_i}^{-1} \ \mathbf{T}_{WB_{i+1}} \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})
\end{align}
$$

Similarly, we use the adjoint to move the exponential twice:

$$
\begin{align}
\mathbf{\tilde{T}}_{B_i B_{i+1}} &=  \mathbf{T}_{WB_i}^{-1}  \text{Exp}(- \text{Ad}_{\mathbf{T}_{WB_i}} {_{B_i}}\mathbf{\eta}_{B_i}) \ \mathbf{T}_{WB_{i+1}} \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})\\
 &=  \mathbf{T}_{WB_i}^{-1} \ \mathbf{T}_{WB_{i+1}} \ \text{Exp}(- \text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \ \text{Ad}_{\mathbf{T}_{WB_i}} {_{B_i}}\mathbf{\eta}_{B_i}) \ \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})
\end{align}
$$

Hence, the following covariance holds for the relative pose assuming independent poses:

$$
\begin{equation}
\Sigma_{B_{i+1}} = \text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \text{Ad}_{\mathbf{T}_{WB_i}} \Sigma_{B_{i}} \text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \text{Ad}_{\mathbf{T}_{WB_i}}^{T} + \Sigma_{B_{i+1}}
\end{equation}
$$

A problem that exist with this expression, however, is that by assuming independence the covariance of the relative poses _will be larger than the covariance of each pose separately_. This is consistent to a 1-dimensional case in which we compute the distribution of the difference of independent Gaussians, in which the mean is the difference while the covariance gets increased. Mangelson et al. showed that if some correlations exists and it is explicitly considered in the computation, the estimates get more accurate and the covariance is not over or underestimated. The corresponding expression is then:

$$
\begin{equation}
\Sigma_{B_{i+1}} = \left(\text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \text{Ad}_{\mathbf{T}_{WB_i}} \right) \Sigma_{B_{i}} \left(\text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \text{Ad}_{\mathbf{T}_{WB_i}}^{T} \right) + \Sigma_{B_{i+1}} - \text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \text{Ad}_{\mathbf{T}_{WB_i}} \Sigma_{B_{i} B_{i+1}} - \Sigma_{B_{i} B_{i+1}}^{T} (\text{Ad}_{\mathbf{T}_{WB_{i+1}}^{-1}} \text{Ad}_{\mathbf{T}_{WB_i}})^{T}
\end{equation}
$$

## Conclusions 

In this post we took a long trip to explain how the covariances are introduced and extracted from the estimation framework. We reviewed how a clear understanding of the conventions (explicit or not) embeded in our factor graph problems is fundamental to make sense of the quantities involved. We introduced the concept of _right-hand_ and _left-hand_ conventions which, while not standard, allowed us to identify different formulations that can be found in the literature. By explicitly stating that GTSAM uses a right-hand convention we could specify the frames used to define the variables, as well the covariance we obtain from the solution via  `Marginals`.

We also showed how the right-hand convention is related to how the composition is defined when working with manifolds, such as rigid-body transformations and rotation matrices. The  **local** and **retract** operations are also defined from the right-hand side, indicating that the increments and the covariances we compute are defined with respect to the base frame.

Additionally, the definition of the local and retract operations also have direct impact on the ordering of the covariance matrices, which also varies depending on the object. We discussed that `Pose2` use a _translation-then-orientation_ convention, while `Pose3` does _orientation-then-translation_. This is particularly important when we want to use GTSAM quantities with different software, such as ROS, which use a _translation-then-orientation_ convention for their 3D pose structures for instance ([`PoseWithCovariance`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html)).

Finally, we showed how the convention is also important to define probability distributions of rigid-body matrices, and how this determine the way the covariances get transformed when manipulating such variables. While this has been shown in related literature, we briefly presented some expressions for inversion, composition and relative poses that are compliant with GTSAM's convention.

## Acknowledgments
I would like to thank again the interesting discussions originated in the [gtsam-users](https://groups.google.com/g/gtsam-users/c/c-BhH8mfqbo/m/7Wsj_nogBAAJ) group. Stefan Gächter guided a rich conversation doing some important questions, and Frank Dellaert motivated the idea of writing a post about it.

Coincidently, similar discussions at the [Dynamic Robot Systems](https://ori.ox.ac.uk/labs/drs/) group at the University of Oxford were aligned with the topics discussed here and facilitated the writing process. Special thanks to Yiduo Wang and Milad Ramezani for our conversations, derivation and testing of the formulas for the covariance of relative poses presented here, Marco Camurri for feedback on the notation, and Maurice Fallon for encouraging to write this post.

