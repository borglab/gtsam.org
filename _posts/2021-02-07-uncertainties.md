---
layout: gtsam-post
title:  "Reducing the uncertainty about the uncertainties"
---

<link rel="stylesheet" href="/assets/css/slideshow.css">

Authors: [Matias Mattamala](https://mmattamala.github.io)

<div style="display:none"> <!-- custom latex commands here -->
  $
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

<!-- - TOC
{:toc} -->

## Introduction
<a name="fg_scratch"></a>
<figure class="center">
  <img src="/assets/images/lqr_control/VE/fg_lqr.png"
    alt="Factor graph structure. The objective factors are marked with dashed lines, and the constrain factors are marked with solid lines." />
    <figcaption><b>Figure 1</b> Factor graph structure for an LQR problem with 3 time steps. The cost factors are marked with dashed lines and the dynamics constraint factors are marked with solid lines.</figcaption>
</figure>
<br />

In this post I will review some general aspects of optimization-based state estimation methods, and how to input and output consistent quantities and uncertainties, i.e, covariance matrices, from them. We will take a (hopefully) comprehensive tour that will cover _why we do the things the way we do_, aiming to clarify some _uncertain_ things about working with covariances. We will see how most of the explanations naturally arise by making explicit the definitions and conventions that sometimes we implicitly assume when using these tools.

This post summarizes and extends some of the interesting discussions we had in the [gtsam-users](https://groups.google.com/g/gtsam-users/c/c-BhH8mfqbo/m/7Wsj_nogBAAJ) group. We hope that such space will continue to bring to the table relevant questions shared by all of us.

## A simple example: a pose graph
As a motivation, we will use a similar pose graph to ones used in other GTSAM examples:

Insert image>_

For now, we will consider that the variables in the graph $\mathbf{x}_i$ correspond to positions $\mathbf{x}_i = (x,y) \in \mathbb{R}^2$. The variables are related by a transition matrix $\mathbf{A}$, as well as relative _measurements_ $\mathbf{b}_i = (b_x, b_y)_i$ obtained from some sensor such as a wheel odometer. We can then establish the following relationships between variables $i$ and $i+1$:

$\mathbf{x}_{i+1} = \mathbf{A} \mathbf{x}_i + \mathbf{b}_i$

However, we now that in reality things do not work that way, and we will usually have errors produced by noise in our sensors. The most common strategy we use to solve this problem is adding some _zero-mean Gaussian noise_ $\eta_i\sim Gaussian(\mathbf{0}_{2\times1}, \Sigma_i)$ to our measurement to model this uncertainty:

$\mathbf{x}_{i+1} = \mathbf{A}\mathbf{x}_i + \delta\mathbf{x}_i + \eta_i$

We can recognize here the typical _motion_ or _process model_ we use in Kalman filter for instance, that describe how our state evolves. We say that the noise we introduced on the right hand side states that our next state $\mathbf{x}_{i+1}$ is gonna be _around_ $\mathbf{A}\mathbf{x}_i + \mathbf{b}_i$, and the covariance matrix $\Sigma_i$ describes a region where we expect $\mathbf{x}_{i+1}$ to lie.

We can also notice that with a bit of manipulation, it is possible to establish the following relationship:

$\eta_i  = \mathbf{x}_{i+1} - \mathbf{A}\mathbf{x}_i - \mathbf{b}_i$

This is an important expression because we know that the left-hand expression distributes as a Gaussian distribution. But since we have an equivalence, the right-hand term must do as well. It is important to note here that what distributes as a Gaussian is neither $\mathbf{x}_i$ nor $\mathbf{x}_{i+1}$, but the difference $(\mathbf{x}_{i+1} - \mathbf{A}\mathbf{x}_i + \mathbf{b}_i)$. This allows us to use the difference as the _factor_ that relates $\mathbf{x}_i$ and $\mathbf{x}_{i+1}$ probabillistically in our factor graph.

### Analyzing the solution
Solving the factor graph previously described is equivalent to solving the following least squares problem under the assumption that all our factors are Gaussian (which is fortunately our case):

$\mathcal{X}^{*} = \argmin \displaystyle\sum_i || \mathbf{x}_{i+1} - \mathbf{A}\mathbf{x}_i + \mathbf{b}_i ||^2_{\Sigma_i}$

This problem is linear, hence solvable in closed form. By differentiating the squared cost and setting it to zero, we end up with the so-called _normal equations_, which are particularly relevant for our posterior analysis:

$\mathbf{A}^{T} \Sigma^{-1} \mathbf{A}\ \mathcal{X}^{*} = \mathbf{A}^{T} \Sigma^{-1} \mathbf{b}$

First point we can notice here is that finding the solution $\mathcal{X}^{*}$ requires to invert the matrix $\mathbf{A}^{T} \Sigma^{-1} \mathbf{A}$, which in general is hard since it can be huge and dense in some parts. However we know that there are clever ways to solve it, such as iSAM and iSAM2 that GTSAM already implements, which are covered in this comprehensive [article](https://www.cc.gatech.edu/~dellaert/pubs/Dellaert17fnt.pdf) by Frank Dellart and Michael Kaess.

Our interest in this matrix, though, known as _Fisher information matrix_ or _Hessian_ (since it approximates the Hessian of the original quadratic cost), is that its inverse $\Sigma^{*} = (\mathbf{A}^{T} \Sigma^{-1} \mathbf{A})^{-1}$ also approximates the covariance of our solution - known as _Laplace approximation_ in machine learning. This is a quite important result because by solving the factor graph we are not only recovering an estimate of the mean of the solution, but also a measure of its uncertainty.

Hence, we can say that after solving the factor graph we can define a probability distribution of the solution, which will be given by the $Gaussian(\mathcal{X}^{*}, \Sigma^{*})$.

The resulting covariance of the solution corresponds to 
**Discuss how the covariances get transformed**


## Getting nonlinear
The previous pose graph was quite simple and probably not applicable for most of our problems. Having linear factors as the previous one is an _impossible dream_ for most of the applications. It is more realistic to think that our state $i+1$ will evolve as a nonlinear function of the state $i$ and the measurements $b_i$:

$\mathbf{x}_{i+1} = f(\mathbf{x}_i, \mathbf{b}_i)$

Despite nonlinear, we can still say that our mapping has some errors involved, which are embeded into a zero-mean Gaussian noise as before:

$\mathbf{x}_{i+1} = f(\mathbf{x}_i, \mathbf{b}_i) + \eta_i$

So a similar expression follows by isolating the noise:

$\eta_i = \mathbf{x}_{i+1} - f(\mathbf{x}_i, \mathbf{b}_i)$

Now are factors are defined by $\mathbf{x}_{i+1} - f(\mathbf{x}_i, \mathbf{b}_i)$, which poses the following _nonlinear least squares_ (NLS) problem:

$\mathcal{X} = \argmin \displaystyle\sum_i || \mathbf{x}_{i+1} - f(\mathbf{x}_i, \mathbf{b}_i) ||^2_{\Sigma_i}$

Since the system is not linear anymore, we cannot solve it in close form. We need to use nonlinear optimization algorithms such as Gauss-Newton or Levenberg-Marquardt. They will try to get us close to our linear dream by linearizing with respect to some _linearization_ or _operation_ point around a guess $\mathcal{\bar{X}}^{k}$ valid at iteration $k$:

$f(\mathbf{x}_i, \mathbf{b}_i) \approx f(\mathbf{\bar{x}}^{k}, \mathbf{b}_i) + \mathbf{H}(\bar{x}^{k})\mathbf{x}_i = \mathbf{b}_k + \mathbf{H}^k \mathbf{x}_i$

Hence the problem becomes:

$\delta\mathcal{X}^{k}= \argmin \displaystyle\sum_i || \mathbf{x}_{i+1} - \mathbf{H}^k\mathbf{x}_i - \mathbf{b}_i ||^2_{\Sigma_i^k}$

It is important to observe here that we are not obtaining the global solution $\mathcal{X}$, but just a small increment $\delta\mathcal{X}$ that will allows to move closer to some minima. This linear problem **can** be solved in closed form as we did before; in fact, the normal equations now become:

$(\mathbf{H}^k)^{T} (\Sigma^{k})^{-1}\ \mathbf{H}^k\ \delta\mathcal{X} = (\mathbf{H}^k)^{T} (\Sigma^{k})^{-1} \mathbf{b}$

The solution $\delta\mathcal{X}$ will be used to update our current solution at iteration $k$ using the update rule $\mathcal{X}^{k+1} = \mathcal{X}^k + \delta\mathcal{X}^{k}$. Here, $\mathcal{X}^{k+1}$ corresponds to the best estimate so far, and can be used as a new _linearization point_ for a next iteration.

Similarly, the expression on the left-hand side $\Sigma^{k+1} = (\mathbf{A}^{T} (\Sigma^{k})^{-1} \mathbf{A})^{-1}$ also corresponds to the Fisher information or Hessian as before, **but with respect to the linearization point**. This is quite important because both the best solution so far $\mathcal{X}^{k+1}$ and its associated covariance $\Sigma^{k+1}$ will change with every iteration of the nonlinear optimization. But understanding this, we still can say that at iteration $k+1$ our best solution will follow a distribution $Gaussian(\mathcal{X}^{k+1}, \Sigma^{k+1})$


## Getting non-Euclidean
So far so good, but we need to admit we were not too honest before when we said that considering nonlinear functions of the variables was everything we needed to model real problems. The previous formulation assumed that the variables we aim to estimate are **vectors**, which is not the case for robotics and computer vision at least.

In robotics, we do not estimate the state of a robot only by their position but also its orientation. Then we say that is its _pose_, i.e, position and orientation together, what matters to define its state.

Representing pose is a tricky thing. We could say _"ok, but let's just append the orientation to the position vector and do everything as before"_ but that does not work in practice. Problem arises when we want to compose two poses $\mathbf{T}_1 = (x_1, y_1, \theta_1)$ and $\mathbf{T}_2 = (x_2, y_2, \theta_2)$. Under the vector assumption, we can write the following expression as the composition of two poses:

$\mathbf{T}_1 + \mathbf{T}_2 = \left[ \begin{matrix} x_1\\ y_1 \\ \theta_1\end{matrix} \right] + \left[ \begin{matrix} x_2\\ y_2 \\ \theta_2\end{matrix} \right] = \left[ \begin{matrix} x_1 + x_2\\ y_1+y_2 \\ \theta_1 + \theta_2\end{matrix} \right]$

This is basically saying that _it does not matter if we start in pose $\mathbf{T}_1$ or $\mathbf{T}_2$, we will end up at the same final pose_ by composing both, because in vector spaces we can commute the elements. **But this does not work in reality, because rotations and translations do not commute**.

So we need a different representation for poses that allow us to describe accurately what we observe in reality. Long story short, we rather prefer to represent poses as $3\times3$ matrices known as _rigid-body transformations_:

$\mathbf{T}_1 = \left[\begin{matrix} \cos{\theta_1} && -\sin{\theta_1} && x_1 \\ \sin{\theta_1} && \cos{\theta_1} && y_1 \\ 0 && 0 && 1\end{matrix} \right] = \left[\begin{matrix} \mathbf{R}_1 && \mathbf{t}_1 \\ 0 && 1\end{matrix}\right]$

Here $\mathbf{R}_1 = \left[ \begin{matrix} \cos{\theta_1} && -\sin{\theta_1}\\ \sin{\theta_1} && \cos{\theta_1}\end{matrix} \right]$ is a 2D rotation matrix, while $\mathbf{t}_1 = \left[ \begin{matrix}x_1 \\ y_1 \end{matrix}\right]$ is a translation vector.
While we are using a $3\times3$ matrix now to represent the pose, its _degrees of freedom_ are still $3$, since it is a function of $(x_1, y_1, \theta_1)$.

Working with transformation matrices is great, because we can now describe the behavior we explained in words before using matrix operations. If we start in pose $\mathbf{T}_1$ and we apply the transformation $\mathbf{T}_2$:

$\mathbf{T}_1 \mathbf{T}_2 = \left[\begin{matrix} \mathbf{R}_1 && \mathbf{t}_1 \\ 0 && 1\end{matrix}\right] \left[\begin{matrix} \mathbf{R}_2 && \mathbf{t}_2 \\ 0 && 1\end{matrix}\right] = \left[\begin{matrix} \mathbf{R}_1 \mathbf{R}_2 && \mathbf{R}_1 \mathbf{t}_2 +  \mathbf{t}_1 \\ 0 && 1\end{matrix}\right]$

Which is different to starting from $\mathbf{T}_2$ and applying $\mathbf{T}_1$:

$\mathbf{T}_2 \mathbf{T}_1 = \left[\begin{matrix} \mathbf{R}_2 && \mathbf{t}_2 \\ 0 && 1\end{matrix}\right] \left[\begin{matrix} \mathbf{R}_1 && \mathbf{t}_1 \\ 0 && 1\end{matrix}\right] = \left[\begin{matrix} \mathbf{R}_2 \mathbf{R}_1 && \mathbf{R}_2 \mathbf{t}_1 +  \mathbf{t}_2 \\ 0 && 1\end{matrix}\right]$

In fact, we now rewrite the same problem as before, but now using our transformation matrices:

$\mathbf{T}_{i+1} = \mathbf{T}_i \ \Delta\mathbf{T}_i$

Here we are saying that the next state $\mathbf{T}_{i+1}$ is gonna be the previous state $\mathbf{T}_{i}$ _plus_ some increment $\Delta\mathbf{T}_i$ given by a sensor -odometry in this case. Please note that since we are now working with transformation matrices, $\Delta\mathbf{T}_i$ is almost everything we need to model the problem more accurately, since it will represent a change in both position and orientation.

Now, **there are two things here that we must discuss** before moving on, which are fundamental to make everything consistent. We will review them carefully now.

### The importance of coordinate frames
While I personally prefer to write the process as we did before, applying the increment on the **right-hand** side:

$\mathbf{T}_{i+1} = \mathbf{T}_i \ \Delta\mathbf{T}_i$

Some people prefer to do it a different way, on the **left-hand** side:

$\mathbf{T}_{i+1} = \Delta\mathbf{T}_i \ \mathbf{T}_i$

These expressions are not equivalent as we already discussed because rigid-body transformations do not commute. However, _both make sense_, under specific conventions. We will be more explicit now by introducing the concept of **reference frames** in our notation.

Let us say that the robot trajectory in space is expressed in a fixed frame -we will called it the _world frame_ $W$. The robot itself defines a coordinate frame in its body, the _body frame_ $B$.

By doing this we are definint the **pose of the robot body $B$ expressed in the world frame $W$ using the right-hand convention** by using the following notation:

$\mathbf{T}_{WB}$

Analogously, using the **left-hand convention**:

$\mathbf{T}_{BW}$

According to this formulation, we can interpret the right-hand convention as a description of the process _from a fixed, world frame_ while the left-hand one does it _from the robot's perspective_. The interesting thing is that we can always go from one to the other by inverting the matrices:

$\mathbf{T}_{WB}^{-1} = \mathbf{T}_{BW}$

So the inverse _swaps_ the subindices, effectively changing our _point-of-view_ to describe the world.

**Using one convention or the other does not matter. The important thing is to stay consistent**. 
In fact, by making the frames explicit in our convention we can always check if we are making mistakes. Let us say we are using the right-hand notation as above, and our odometer is providing increments _relative to the previous body state_. Our increments $\Delta\mathbf{T}_i$ will be $\Delta\mathbf{T}_{B_{i} B_{i+1} }$, i.e, it's a transformation that describes the pose of the body at time $i+1$ relative to the previous one $i$.

Hence, the actual pose of the body at time $i+1$ expressed in the world frame $W$ is:

$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1} }$

We now this is right because the inner indices cancel each other, effectively representing the pose at $i+1$ in the frame we want:

$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{W \cancel{B_i}} \ \Delta\mathbf{T}_{\cancel{B_{i}} B_{i+1} }$

If we were actually using the left-hand convention though, we would need to invert the measurement:

$\Delta\mathbf{T}_{B_{i+1} B_{i}} = \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}$

so we can apply the transformation on the left-hand side:

$\mathbf{T}_{B_{i+1}W} = \Delta\mathbf{T}_{B_{i+1} B_{i}} \mathbf{T}_{B_i W}$


**In GTSAM we use the right-hand convention**. It is important to have this in mind because all the operations that are implemented follow this. As a matter of fact, 3D computer vision has generally used left-hand convention because it is straightforward to apply the projection models:

$_I\mathbf{p} = {_{I}}\mathbf{K}_{IC}\ \mathbf{T}_{CW}\ {_{W}}\mathbf{P}$

Here we made explicit all the frames involved in the transformations we usually can find in textbooks: An homogeneous point ${_{W}}\mathbf{P}$ expressed in the world frame, is transformed by the _extrinsic calibration matrix_ given by the rigid body transformation $\mathbf{T}_{CW}$ which represents the world $W$ in the camera frame $C$, producing a vector ${_{C}}\mathbf{P}$ in the camera frame. This is projected onto the image frame $I$ by means of the intrinsic calibration matrix ${_{I}}\mathbf{K}_{IC}$, producing the vector $_I\mathbf{p}$, expressed in the image frame.

Since in GTSAM our extrinsic calibration is defined the other way, i.e $\mathbf{T}_{WC}$, the implementation of the `CalibratedCamera` handles it by [properly inverting the matrix before doing the projection](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/CalibratedCamera.cpp#L120) as we would expect.

### They are not just matrices, they are _manifolds_

The second important point we need to discuss, is that while rigid-body transformations are nice, the new expression we have to represent the process presents some challenges:

$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1} }$

In first place, we need to figure out a way to include the _noise_ term $\eta_i$ we used before to handle the uncertainties about our measurement $\Delta\mathbf{T}_{B_{i} B_{i+1} }$, which was also the trick we used to generate the Gaussian factors we use in our factor graph. For now, we will say that the noise will be given by a matrix $\mathbf{N}_i$, which holds the following:

$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}} \mathbf{N}_i$


Secondly, assuming the noise $\mathbf{N}_i$ was defined somehow, we can isolate it as we did before. However, we need to use matrix operations now:

$\mathbf{N}_i = (\mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}})^{-1} \mathbf{T}_{WB_{i+1}}$

and we can manipulate it a bit for clarity:

$\mathbf{N}_i = (\Delta\mathbf{T}_{B_{i} B_{i+1}})^{-1} (\mathbf{T}_{WB_i})^{-1} \mathbf{T}_{WB_{i+1}}$

If we do the subindex cancelation trick we did before, we can confirm that the error is defined _in frame $B_{i+1}$ and expressed in frame $B_{i+1}$_. This may seen counterintuitive, but in fact matches what we should expect:

INSERT FIGURE

However, the error is still a matrix, which is impossible to include as a factor in the framework we have built so far. We cannot compute a _vector error_ as we did before, nor we are completely sure that the matrix $\mathbf{N}_i$ follows some sort of Gaussian distribution.

#### Manifolds
Here is where the concept of **manifold** comes to solve our problems. Rigid-body transformations (`Pose3` and `Pose2` in GTSAM), rotation matrices (`Rot2` and `Rot3`), quaternions and even vector spaces (`Point2` and `Point3`) are _differentiable manifolds_. This means that in spite of they do not behave as Euclidean spaces at a global scale, they can be _locally approximated_ as such by using local vector spaces called **tangent spaces**. The main advantage of analyzing all these objects from the manifold perspective is that we can build general algorithms based on common principles that apply to all of them.

In order to work with all the previous objects we mentioned -rotation matrices, rigid-body transformations, quaternions-, we need to define some basic operations that resemble the vector case that allows us to generalize the framework. These concepts [have been discussed in previous posts](https://gtsam.org/notes/GTSAM-Concepts.html) at the implementation level:

1. **_Composition_**: How to compose 2 objects from the same manifold. It also needs associativity properties. It is similar to an _addition_ operation.
2. **_Between_**: Computes the difference between 2 objects from the same manifold. Similar to a _subtraction_ operation.
3. **_Identity_**: An identity operator under the composition/between.
4. **_Inverse_**: An element that when composed with its inverse becomes the identity.
5. **_Local_**: An operation that maps elements from the manifold to the tangent space.
6. **_Retract_**: The oppositte operation: mapping from the tangent space back to the manifold.

The first 4 are basic properties we need to define a **group**. In fact, the _between_ operation is simply a by-product of having the composition, inverse and identity well-defined. We can also notice that they are operations we already used before in this post when working with rigid-body matrices. The only difference is that now we defined them in a more general way.

Some authors define special operators for the composition/between operations, such as _box-plus_ $\boxplus$ for _composition_ and _box-minus_ $\boxminus$ for _between_ as done by [Hertzberg et al.](https://arxiv.org/abs/1107.1119) for general manifolds, and [Bloesch et al.](https://arxiv.org/abs/1606.05285) and the [Kindr library](https://github.com/ANYbotics/kindr) for rotations. However, they are also implicitly assuming a _left or right-hand_ convention when doing so, what makes important to understand clearly the choices of each author. [Solà et al](https://arxiv.org/abs/1812.01537) make the difference explicit by defining _left-_$\oplus$ and _right-_$\oplus$ for composition using _left-hand convention_ of _right-hand convention_ respectively. We recall again that in GTSAM we use the right convention.

#### Local and retract
Regarding the last 2 properties, **local** and **retract** operations are the key to work with manifolds. As we briefly mentioned before, objects such as rotation matrices and rigid-body transformations are difficult to manipulate in the estimation framework because they are matrices. A 3D rotation matrix $\mathbf{R}$ represents 3 orientations with respect to a reference frame but, in raw terms, they are using 9 values to do so, which seems to _overparametrize_ the object. However, the constraints that define a rotation matrix -and consequently the manifold- such as orthonormality $\mathbf{R}^{T}\mathbf{R} = \mathbf{I}$ and $\text{det}(\mathbf{R}) = 1$ make the inherent dimensionality of the rotation still 3. Which is exactly the dimensionality of the tangent spaces that can be defined over the manifold.

This is what makes working with manifolds so convenient: All the constraints that are part of the definition of the object are naturally handled, and we can work in local vector spaces using their _inherent_ dimension. The same happens for rigid-body transformations (6 dimensions represented by a 16 elements matrix) or quaternions (3 orientations represented by a 4D vector).

The mapping **from the manifold to the tangent space** is given by the **local** operator. Conversely, we can go **from the tangent space to the manifold** by using the **retract**.

People familiar with Lie groups will ask what are the differences with them: Lie groups are also defined as groups that are also differentiable manifolds. Indeed, when working with rigid-body transformations for instance, we use the same operations defined for the _Special Euclidean group_ $\text{SE(3)}$ as used in our previous examples:

1. **_Composition_**: Matrix multiplication $\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1} }$
2. **_Between_**: $\Delta\mathbf{T}_{B_{i} B_{i+1}} = (\mathbf{T}_{WB_i})^{-1} \  \mathbf{T}_{WB_{i+1}}$
3. **_Identity_**: Identity matrix $\mathbf{I}_W$
4. **_Inverse_**: Matrix inverse $(\mathbf{T}_{WB_i})^{-1} = \mathbf{T}_{B_i W}$
5. **_Local_**: We use the _logarithm map_ of $\text{SE(3)}$: $_W\mathbf{\xi}_{W} = \text{Log}(\mathbf{T}_{WB_i} )$ 
6. **_Retract_**: We use the _exponential map_ of $\text{SE(3)}$: $\mathbf{T}_{WB_i} = \text{Exp}(_W\mathbf{\xi}_{W})$

The main difference is that by using the general concept of [_retraction_](https://press.princeton.edu/absil) we can use **alternative definitions to go from the tangent space to the manifold**, that can be more efficient than the exponential map when solving optimization problems.

It is also important to notice that **the reference frames are preserved when applying the local and retract operations**. For instance, when using the _local_ operation we defined using the logarithm map of  $\text{SE(3)}$, we obtain a vector $_W\mathbf{\xi}_{W} \in \mathbb{R}^{6}$ (sometimes also called _tangent vector_ or _twist_), which is defined on the tangent space _centered at the world frame_ in this case:

$_W\mathbf{\xi}_{W} = \text{Log}(\mathbf{T}_{WB_i} )$ 

The same property holds to _add incremental changes to a transformation_:

$\mathbf{T}_{WB_i+1} = \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i})$

In this case we added an increment from the base frame at time $i$, that represents the new pose at time $i+1$. Please note that the increments are defined with respect to a reference frame, but they do not require to specify the resulting frame. Their meaning (representing a new pose at time $i+1$) is something we define but is not explicit in the formulation.

 **The incremental formulation via retractions is particularly convenient when we have local (base frame) velocity measurements** $(_B\omega_B, _B v_B)$, with $_B\omega_B \in \mathbb{R}^{3}, _B v_B \in \mathbb{R}^{3}$  and we want to do dead reckoning:

$\mathbf{T}_{WB_i+1} = \mathbf{T}_{WB_i} \text{Exp}\left( \left[\begin{matrix} _B\omega_B \\ _B v_B \end{matrix}\right] \delta t \right)$

The product $\left[\begin{matrix} _B\omega_B \ \delta t \\ _B v_B \ \delta t\end{matrix}\right]$  represents the tangent vector that we map onto the manifold by means of the $\text{SE(3)}$ retraction. 

**We need to be careful about the convention of the retraction/local operation** (yes, more conventions again). Having clarity about definition that every software defines for these operations (even implicitly) is fundamental to make sense of the quantities we put into our estimation problems and the estimates we extract. For instance, the definition of the $\text{SE(3)}$ retraction we presented, which matches `Pose3` in GTSAM, uses an _orientation-then-position_ convention, i.e, the 6D tangent vector has orientation in the first 3 coordinates, and position in the last 3. On the other hand, `Pose2` uses _position-then-orientation_ $(x, y, \theta)$ for [historical reasons](https://github.com/borglab/gtsam/issues/160#issuecomment-562161665).

#### Adjoints
Lastly, we need to discuss an operator that will be useful to operate covariances correctly when we return to our original estimation problem. To introduce it, let us consider the example before in which we added an incremental change in the base frame:

$\mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i})$

It is possible to found an incremental change applied at the world frame $_{W}\mathbf{\xi}_{W}$ that can lead to the same result:

$\text{Exp}( _{W}\mathbf{\xi}_{W}) \mathbf{T}_{WB_i} = \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i})$

This incremental change $_{W}\mathbf{\xi}_{W}$ must be given by:

$\text{Exp}( _{W}\mathbf{\xi}_{W}) = \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\xi}_{B_i}) \mathbf{T}_{WB_i}^{-1}$

The expression on the right-hand side is known as the _adjoint action_ of $\text{SE(3)}$. This relates increments applied on the left to ones applied on the right. For our purposes, it is useful to use the alternative expression ([Solà et al](https://arxiv.org/abs/1812.01537) give a more complete derivation):

$\text{Exp}( _{W}\mathbf{\xi}_{W}) \mathbf{T}_{WB_i} = \mathbf{T}_{WB_i} \text{Exp}( \text{Ad}_{T_{WB_i}^{-1}}  {_{W}}\mathbf{\xi}_{W})$

where $\text{Ad}_{T_{WB_i}^{-1}}$ is known as **the adjoint of $T_{WB_i}^{-1}$**. The adjoint acts over elements of the tangent space directly, changing their reference frame. Please note that the same subindex cancelation applies here, so we can confirm that the transformations are correctly defined.

We can also interpret this as a way to _move_ increments applied on the left-hand side to the right-hand side, which is particularly useful to keep the right-hand convention used in GTSAM consistent.

## Bringing everything together
Now we can return to our original estimation problem using rigid-body transformations. Let us recall that we defined the following process model given by odometry measurements:

$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}} \mathbf{N}_i$

We reported two problems before:
1. We needed to define the noise $\mathbf{N}_i$ as a Gaussian noise, but it was a $6\times6$ matrix
2. When isolating the noise, we ended up with a matrix expression, not vector one as we needed to use the estimation framework we presented before.

The first problem can be solved using the tools we just defined by defining _a zero-mean Gaussian in the tangent space of $\text{SE(3)}$ and retracting it onto the manifold_:

$\mathbf{T}_{WB_{i+1}} = \mathbf{T}_{WB_i} \ \Delta\mathbf{T}_{B_{i} B_{i+1}} \text{Exp}( _{B_i}\mathbf{\eta}_{B_i})$

where we have defined $\eta_{B_i} \sim Gaussian(\mathbf{0}_{6\times1}, \Sigma_i)$, indicating that, in order to make sense of our conventions, **the covariances we use for the factor must be defined in the base frame**. In fact, we can use the same procedure to define Gaussians for any transformation $\mathbf{T}_{WB}$:

$\mathbf{\tilde{T}}_{WB} = \mathbf{T}_{WB} \text{Exp}( _{B}\mathbf{\eta}_{B})$

This definition has been widely used in the past in estimation problems and matches the one used by GTSAM. In the literature, however, definitions differ in how the Gaussian is retracted (left-hand or right-hand), similarly to the issue with the composition operation. [Barfoot and Furgale (2014, left-hand convention)](http://ncfrn.cim.mcgill.ca/members/pubs/barfoot_tro14.pdf), [Forster et al (2017), right-hand convention](https://arxiv.org/abs/1512.02363), and [Mangelson et al. (2020), left-hand convention](https://arxiv.org/abs/1906.07795) are some examples. Other definitions to define probability distributions on manifolds include [Calinon (2020)](https://arxiv.org/abs/1909.05946) and [Lee et al. (2008)](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.175.5054&rep=rep1&type=pdf), please refer their work for further details.

Having solved the first problem, we can now focus on the next. We can isolate the noise as we did before, which holds:

$\text{Exp}( _{i+1}\mathbf{\eta}_{B_{i+1}}) = \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \  \mathbf{T}_{WB_{i+1}}$

However, now we can apply the **local** operator of the manifold, which corresponds to the _logarithm map_ for $\text{SE(3)}$:

$_{B_{i+1}}\mathbf{\eta}_{B_{i+1}} = \text{Log}\left( \Delta\mathbf{T}_{B_{i} B_{i+1}}^{-1}\ \mathbf{T}_{WB_i}^{-1} \  \mathbf{T}_{WB_{i+1}} \right)$

Since the noise is defined in the tangent space, it is a vector expression in $\mathbb{R}^{6}$, as is the right side. Both are defined as zero-mean Gaussians, hence the right-hand side can be used as a proper factor in our estimation framework. In fact, the expression on the righ-hand side is _exactly_ the same used in GTSAM to define the [`BetweenFactor`](https://github.com/devbharat/gtsam/blob/master/gtsam/slam/BetweenFactor.h#L90).

Additionally, the factor is now a nonlinear vector expression that can be solved using the nonlinear optimization techniques we presented before. However, there are subtle differences that we must clarify.

First, since the factor defines a residual in the tangent space at the current linearization point, the optimization itself is executed **in the tangent space defined in the current linearization point_**. This means that when we linearize the factors and build the normal equations, the increment ${_{B_i}}\delta\mathbf{T}^{k}$ we compute lies in the tangent space.

For this reason, we need to update the variables _back onto the manifold_ using the retraction:

$\mathbf{\tilde{T}}_{WB_i}^{k+1} = \mathbf{T}_{WB_i}^k \text{Exp}( {_{B_i}}\delta\mathbf{T}^{k} )$

The second important point, is that the covariance that we can recover from the information matrix, **will be also defined in the tangent space around the linearization point, following the convention of the retraction**. It means that if our information matrix at the current linearization point is given by:

$\Sigma^{k+1} = (\mathbf{A}^{T} (\Sigma^{k})^{-1} \mathbf{A})^{-1}$

Then, the corresponding distribution solution around the current linearization point $\mathbf{T}_{WB_i}^{k+1}$ will be given by:

$Gaussian(\mathbf{T}_{WB_i}^{k+1}, \Sigma^{k+1}) = \mathbf{T}_{WB_i}^{k+1} \text{Exp}( {_{B_i}}\eta_{B_i}^{k+1} )$

where ${_{B_i}}\eta_{B_i}^{k+1} \sim Gaussian(\mathbf{0}_{6\times1}, \Sigma^{k+1})$. As a consequence of the convention of the retraction, **the resulting covariance is expressed in the base frame as well**.


## Playing with covariances
We took a long trip to explain how the covariances are introduced and extracted from the estimation framework. We discussed that the choice of the convention when working with the manifold objects (left or right), as well as a clear definition of the frames are the key to understand the estimates we obtain as well as their covariances.

In this last section, we would like to discuss some consequences of the definitions, and how they can be used to obtain other expressions that can be useful in our estimation problems. We will focus on $\text{SE(3)}$, but similar definitions should apply for other manifolds since they mainly rely on a definition of the adjoint.

Most of this expressions have been already shown in the literature by [Barfoot and Furgale (2014)](http://ncfrn.cim.mcgill.ca/members/pubs/barfoot_tro14.pdf) and [Mangelson et al. (2020)](https://arxiv.org/abs/1906.07795) but since they follow the left-hand convention they are not straightforward to use with GTSAM. 

### Distribution of the inverse
Let us consider we have the distribution:

$\mathbf{\tilde{T}}_{WB} = \mathbf{T}_{WB} \text{Exp}( _{B}\mathbf{\eta}_{B})$

with $_{B}\mathbf{\eta}_{B}$ zero-mean Gaussian noise with covariance $\Sigma_{B}$ as before. The distribution of the inverse can be computed by inverting the expression:

$(\mathbf{\tilde{T}}_{WB})^{-1} = (\mathbf{T}_{WB} \text{Exp}( _{B}\mathbf{\eta}_{B}) )^{-1}$

$(\mathbf{\tilde{T}}_{WB})^{-1} = (\text{Exp}( _{B}\mathbf{\eta}_{B}) )^{-1}\ \mathbf{T}_{WB}^{-1}$

$(\mathbf{\tilde{T}}_{WB})^{-1} = \text{Exp}(- _{B}\mathbf{\eta}_{B}) \ \mathbf{T}_{WB}^{-1}$

However, the _noise_ is defined on the left, which is inconvenient to be consistent with left-hand convention. We can move it to the right using the adjoint:

$(\mathbf{\tilde{T}}_{WB})^{-1} = \ \mathbf{T}_{WB}^{-1}\ \text{Exp}(- \text{Ad}_{\mathbf{T}_{WB}} {_{B}}\mathbf{\eta}_{B})$

This is a proper distribution following the right-hand convention. The covariance of the inverse will be given by:

$\Sigma_{W} = \text{Ad}_{\mathbf{T}_{WB}} \Sigma_B \text{Ad}_{\mathbf{T}_{WB}}^{T}$

To understand how the covariance gets transformed, however, we recommend to refer to [Mangelson et al. (2020)](https://arxiv.org/abs/1906.07795) for a detailed explanation.

### Distribution of the composition
A similar procedure can be followed to compute the covariance of the composition of poses. 

$\mathbf{\tilde{T}}_{WB_i} = \mathbf{\tilde{T}}_{WB_i} \mathbf{\tilde{T}}_{B_i B_{i+1}}$

$\mathbf{\tilde{T}}_{WB_i} = \mathbf{T}_{WB_i} \text{Exp}( _{B_i}\mathbf{\eta}_{B_i})\ \mathbf{T}_{B_i B_{i+1}} \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})$

Analogously, we need to _move_ the noise $_{B_i}\mathbf{\eta}_{B_i}$ to the right, so as to have _the transformations to the left, and the noises to the right_. We can use the adjoint again:

$\mathbf{\tilde{T}}_{WB_i} = \mathbf{T}_{WB_i} \ \mathbf{T}_{B_i B_{i+1}} \text{Exp}(\text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} {_{B_i}}\mathbf{\eta}_{B_i})\  \text{Exp}( _{B_{i+1}}\mathbf{\eta}_{B_{i+1}})$

However, we cannot combine the exponentials because that would assume commutativity that does not hold for $\text{SE(3)}$ as we discussed previously. However, it is possible to use some approximations (also discussed in Mangelson's) to end up with the following expressions for the covariance of the composition:

$\Sigma_{B_{i+1}} = \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} \Sigma_{B_{i}} \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}}^{T} + \Sigma_{B_{i+1}}$

Additionally, if we consider that the poses are correlated, i.e, their joint distribution covariance is given by:

$\left[\begin{matrix} \Sigma_{B_{i}} & \Sigma_{B_{i}, B_{i+1}} \\ \Sigma_{B_{i}, B_{i+1}}^{T} & \Sigma_{B_{i+1}} \end{matrix}\right]$

Then, the distribution of the composition is:

$\Sigma_{B_{i+1}} = \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} \Sigma_{B_{i}} \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}}^{T} + \Sigma_{B_{i+1}} +  \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}} \Sigma_{B_{i}, B_{i+1}} +  \Sigma_{B_{i}, B_{i+1}}^{T} \text{Ad}_{\mathbf{T}_{B_i B_{i+1}}^{-1}}^{T}$


### Distribution of the relative transformation
Computing the distribution of relative poses follows the same procedure:

$\mathbf{\tilde{T}}_{B_i B_{i+1}} = \mathbf{\tilde{T}}_{W B_{i}}^{-1} \mathbf{\tilde{T}}_{WB_{i+1}}$

$\mathbf{\tilde{T}}_{B_i B_{i+1}} = \mathbf{\tilde{T}}_{W B_{i}}^{-1} \mathbf{\tilde{T}}_{WB_{i+1}}$



<!-- ********************** BEGIN VARIABLE ELIMINATION SLIDESHOW ********************** -->
<!-- Slideshow container, based on https://www.w3schools.com/howto/howto_js_slideshow.asp -->
<div class="slideshow-container">
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">2 / 3</div> -->
    <a name="fig_eliminate_x_a"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide1.png"
        alt="Elimination of state $x_2$" />
        <figcaption><b>Figure 2a</b> Elimination of state $x_2$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">2 / 3</div> -->
    <a name="fig_eliminate_x_b"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide2.png"
        alt="Elimination of state $x_2$" />
        <figcaption><b>Figure 2b</b> Elimination of state $x_2$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_eliminate_u_a"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide3.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 3a</b> Elimination of state $u_1$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_eliminate_u_b"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide4.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 3b</b> Elimination of state $u_1$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_merge_factor"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide5.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 3c</b> Cost-to-go at $x_1$ is the sum of the two unary factors on $x_1$ (green)</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide6.png"
        alt="Bayes net" />
        <figcaption><b>Figure 4a</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <!-- <div class="mySlides 0" style="text-align: center;">
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide7.png"
        alt="Bayes net" />
        <figcaption><b>Figure 4b</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div> -->
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide8.png"
        alt="Bayes net" />
        <figcaption><b>Figure 4b</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide10.png"
        alt="Bayes net" />
        <figcaption><b>Figure 4c</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/Elimination/cropped_Slide11.png"
        alt="Bayes net" />
        <figcaption><b>Figure 4d</b> Completed Bayes net</figcaption>
    </figure>
  </div>
  <!-- Next and previous buttons -->
  <a class="prev" onclick="plusSlides(-1,0)">&#10094;</a>
  <a class="next" onclick="plusSlides(1,0)">&#10095;</a>

</div> <!-- slideshow-container -->

<!-- The dots/circles -->
<div style="text-align:center">
  <span class="dot 0" onclick="currentSlide(1,0)"></span>
  <span class="dot 0" onclick="currentSlide(2,0)"></span>
  <span class="dot 0" onclick="currentSlide(3,0)"></span>
  <span class="dot 0" onclick="currentSlide(4,0)"></span>
  <span class="dot 0" onclick="currentSlide(5,0)"></span>
  <span class="dot 0" onclick="currentSlide(6,0)"></span>
  <span class="dot 0" onclick="currentSlide(7,0)"></span>
  <span class="dot 0" onclick="currentSlide(8,0)"></span>
  <span class="dot 0" onclick="currentSlide(9,0)"></span>
  <!-- <span class="dot 0" onclick="currentSlide(10,0)"></span> -->
</div>

<!-- this css is to make the scroll bar disappear when the mouse isn't over the scrollable div...
Taken from my website: https://github.com/gchenfc/gerrysworld2/blob/master/css/activity.css -->
<style>
.scrollablecontent::-webkit-scrollbar {
    width: 5px;
    height: 12px;
}
.scrollablecontent::-webkit-scrollbar-track {
    background: transparent;
}
.scrollablecontent::-webkit-scrollbar-thumb {
    background: #ddd;
    visibility:hidden;
}
.scrollablecontent:hover::-webkit-scrollbar-thumb {
    visibility:visible;
}
/* .slideout {
  width: 400px;
  height: auto;
  overflow: hidden;
  background: orange;
  margin: 0 auto;
  transition: height 0.4s linear;
}
.hide {
    height: 0;
} */
</style>
<!-- ********************** END VARIABLE ELIMINATION SLIDESHOW ********************** -->

<!-- ************************ BEGIN SCROLLABLE ELIMINATION DESCRIPTION ************************ -->
<div class="scrollablecontent" markdown="1" id="sec:elim_scrollable"
    style="overflow-x: hidden; background-color:rgba(0,0,0,0.05); padding:0 8px; margin-bottom: 10px;">
<!-- ************** STATE ************** -->
<div markdown="1" id="sec:elim_state_div" class="slideout">
<a id="sec:elim_state"></a>
### Eliminate a State
Let us start at the last state, $x_2$. Gathering the two factors (marked in
red [Figure 2a](#fig_eliminate_x_a){:onclick="currentSlide(1,0)"}), we have \eqref{eq:potential} the objective function $\phi_1$, and \eqref{eq:constrain} the constraint equation on $x_2$, $u_1$ and $x_1$:

\begin{equation} \phi_1(x_2) = x_2^T Q x_2 \label{eq:potential} \end{equation}

\begin{equation} x_2 = Ax_1 + Bu_1 \label{eq:constrain} \end{equation}

By substituting $x_2$ from the dynamics constraint \eqref{eq:constrain} into the objective function
\eqref{eq:potential}, we create a new factor representing
the cost of state $x_2$ as a function of $x_1$ and $u_1$:

\begin{equation} \phi_2(x_1, u_1) = (Ax_1 + Bu_1)^T Q (Ax_1 + Bu_1)
\label{eq:potential_simplified} \end{equation}

The resulting factor graph is illustrated in [Figure 2b](#fig_eliminate_x_b){:onclick="currentSlide(2,0)"}.  Note that the 
dynamics constraint is now represented by the bayes net factors shown as gray arrows.

To summarize, we used the dynamics constraint to eliminate variable
$x_2$ and the two factors marked in red, and we replaced them with a new binary cost factor on $x_1$
and $u_1$, marked in blue. 
<p align="right" style="background-color: rgba(0,0,0,0.1);
    margin: 0px -8px 0 -8px;
    padding: 0 8px;"><a onclick="currentSlide(3,0)">next >></a>
</p>

</div>
<!-- ************** CONTROL ************** -->
<div markdown="1" id="sec:elim_ctrl_div" class="slideout">
<a id="sec:elim_ctrl"></a>
### Eliminate a Control
<!-- Now \eqref{eq:potential_simplified} defines an (unnormalized) joint
Gaussian density on variables $x_1$ and $u_1$.  -->
To eliminate $u_1$, we seek to replace the two factors marked red in [Figure 3a](#fig_eliminate_u_a){:onclick="currentSlide(3,0)"}
with a new cost factor on $x_1$ and an equation for the optimal control $$u_1^*(x_1)$$.

Adding the control cost to \eqref{eq:potential_simplified}, the combined cost of the
two red factors in [Figure 3a](#fig_eliminate_u_a){:onclick="currentSlide(3,0)"} is given by:

\begin{equation} \phi_3(x_1, u_1) = u_1^TRu_1 + (Ax_1 + Bu_1)^T Q (Ax_1 + Bu_1)
\label{eq:potential_u1} \end{equation}

$\phi_3$ is sometimes referred to as the *optimal action value function* and we seek to minimize it over $u_1$.
We do so by
setting the derivative of \eqref{eq:potential_u1} wrt $u_1$ to zero
<!-- (detailed calculation in the [Appendix](#eliminate-u_1)),  -->
yielding the expression for the optimal control input $u_1^*$ as 

\\[ \begin{align} 
u_1^*(x_1) &= \argmin\limits_{u_1}\phi_3(x_1, u_1) \nonumber \\\\ 
&= -(R+B^TQB)^{-1}B^TQAx_1 \label{eq:control_law} \\\\ 
&= K_1x_1 \nonumber
\end{align} \\]

where $K_1\coloneqq -(R+B^TQB)^{-1}B^TQA$.

Finally, we substitute the expression of our optimal control, $$u_1^* = K_1x_1$$,
into our potential \eqref{eq:potential_u1}
<!-- (detailed calculation in the [Appendix](#marginalization-cost-on-x_1)) -->
to obtain a new unary cost factor on $x_1$:

\begin{align}
    \phi_4(x_1) &= \phi_3(x_1, u_1^*(x_1)) \nonumber \\\\ 
        &= (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + BKx_1) \nonumber \\\\ 
        &= x_1^T(A^TQA-K_1^TB^TQA)x_1 \label{eq:potential_x1}
\end{align}
Note that we simplified $K_1^TRK_1 + K_1^TB^TQBK_1 = -K_1^TB^TQA$ by substituting in for $K_1$ using
\eqref{eq:control_law}.

The resulting factor graph is illustrated in [Figure 3b](#fig_eliminate_u_b){:onclick="currentSlide(4,0)"}.

For convenience, we will also define $P_k$ where $x_k^TP_kx_k$ represents the aggregate of the two unary costs on $x_k$.  In the case of $P_1$,
\begin{align}
    x_1^TP_1x_1 &= x_1^TQx_1 + \phi_4(x_1) \nonumber
\end{align}
is the aggregation of the two unary factors labeled in green in [Figure 3c](#fig_merge_factor){:onclick="currentSlide(5,0)"}. 
<p style="background-color: rgba(0,0,0,0.1);
    margin: 0px -8px 0 -8px;
    padding: 0 8px;">
<span align="left"><a onclick="currentSlide(2,0)"><< prev</a><span style="float:right"><a onclick="currentSlide(6,0)">next >></a></span></span>
</p>
</div>
<!-- ************** BAYES NET ************** -->
<div markdown="1" id="sec:elim_bayes_div" class="slideout">
<a id="sec:elim_bayes"></a>
### Turning into a Bayes Network
By eliminating all the variables from right to left, we can get a Bayes network
as shown in [Figure 4d](#fig_bayes_net){:onclick="currentSlide(9,0)"}. Each time we eliminate a state
and control, we simply repeat the steps in [Eliminate a state](#eliminate-a-state) and [Eliminate a control](#eliminate-a-control): we express the state $x_{k+1}$ with the dynamics model, then find the optimal control $u_k$ as
a function of state $x_k$.

Eliminating a general state, $x_{k+1}$, and control $u_k$, we obtain the recurrence relations:

\begin{equation} \boxed{K_k = -(R+B^TP_{k+1}B)^{-1}B^TP_{k+1}A} \label{eq:control_update_k} \end{equation}

\begin{equation} \boxed{P_k = Q+A^TP_{k+1}A - K_k^TB^TP_{k+1}A} \label{eq:cost_update_k} \end{equation}

with $P_{T}=Q$ is the cost at the last time step.

The final Bayes net in [Figure 4d](#fig_bayes_net){:onclick="currentSlide(9,0)"} shows graphically the optimal control law:
\begin{equation} \boxed{u^*_k = K_k x_k} \end{equation}
<p style="background-color: rgba(0,0,0,0.1);
    margin: 0px -8px 0 -8px;
    padding: 0 8px;">
<span align="left"><a onclick="currentSlide(5,0)"><< prev</a></span>
</p>
</div>
</div> <!-- scrollablecontent -->
<!-- ************************ END SCROLLABLE ELIMINATION DESCRIPTION ************************ -->

## Intuition
<!-- ************** Value Function ************** -->
<a name="LQR_example"></a>
<figure class="center" style="width:90%;padding:10px">
  <a href="/assets/images/lqr_control/LQR_FGvsRicatti.png"><img src="/assets/images/lqr_control/LQR_FGvsRicatti.png"
    alt="Comparison between LQR control as solved by factor graphs and by the Ricatti Equation. (they are the same)"
    style="margin-bottom:10px; max-width: 110%; margin-left: -5%;"/></a>
  <figcaption><b>Figure 5</b> Example LQR control solutions as solved by factor graphs (middle) and the traditional Discrete Algebraic Ricatti Equations (right).  The optimal control gains and cost-to-go factors are compared (left).  All plots show exact agreement between factor graph and Ricatti equation solutions.</figcaption>
</figure>

We introduce the **cost-to-go** (also known as *return cost*, *optimal state value function*, or simply *value function*) as $V_k(x) \coloneqq x^TP_kx$ which intuitively represents *the total cost that will be accrued from here on out, assuming optimal control*.

In our factor graph representation, it is becomes obvious that $V_k(x)$ corresponds to the total cost at and after the state $x_k$ assuming optimal control because we eliminate variables backwards in time with the objective of minimizing cost.
Eliminating a state just re-expresses the future cost in terms of prior states/controls.  Each time we eliminate a control, $u$, the future cost is recalculated assuming optimal control (i.e. $\phi_4(x) = \phi_3(x, u^*)$).

This "cost-to-go" is depicted as a heatmap in [Figure 5](#LQR_example).
The heat maps depict the $V_k$ showing that the cost is high when $x$ is far from 0, but also showing that after iterating sufficient far backwards in time, $V_k(x)$ begins to converge.  That is to say, the $V_0(x)$ is very similar for $T=30$ and $T=100$.
Similarly, the leftmost plot of [Figure 5](#LQR_example) depicts $K_k$ and $P_k$ and shows that they (predictably) converge as well.

This convergence allows us to see that we can extend to the [infinite horizon LQR problem](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Infinite-horizon,_discrete-time_LQR) (continued in the next section).

<!-- The factor graph representation also gives us insight to the equation for the optimal gain matrix $K_k$ from
\eqref{eq:control_update_k}.
The optimal control, $K_k$, should attempt to balance (a) the unary factor $u_k^TRu_k$ representing the cost of executing a control action and (b) the binary factor $(Ax_k+Bu_k)^TP_{k+1}(Ax_k+Bu_k)$ representing the future cost of the control action.

The binary factor consists of two terms
represents a balance between achieving a small "cost-to-go" next time step ($B^TP_{k+1}B$) and exerting a small
amount of control this time step ($R$). -->

## Equivalence to the Ricatti Equation

In traditional descriptions of discrete, finite-horizon LQR (i.e. [Chow](https://www.amazon.com/Analysis-Control-Dynamic-Economic-Systems/dp/0898749697), [Kirk](https://pdfs.semanticscholar.org/9777/06d1dc022280f47a2c67c646e85f38d88fe2.pdf#page=86), [Stanford](https://stanford.edu/class/ee363/lectures/dlqr.pdf)), the control law and cost function are given by

\\[ u_k = K_kx_k \\]

\begin{equation} K_k = -(R+B^TP_{k+1}B)^{-1}B^TP_{k+1}A \label{eq:control_update_k_ricatti} \end{equation}

\begin{equation} P_k = Q+A^TP_{k+1}A - K_k^TB^TP_{k+1}A \label{eq:cost_update_k_ricatti} \end{equation}

with $P_k$ commonly referred to as the solution to the **dynamic Ricatti equation** and $P_T=Q$ is the
value of the Ricatti function at the final time step.
\eqref{eq:control_update_k_ricatti} and \eqref{eq:cost_update_k_ricatti} correspond to
the same results as we derived in \eqref{eq:control_update_k} and \eqref{eq:cost_update_k}
respectively.

Recall that $P_0$ and $K_0$ appear to converge as the number of time steps grows.  They will approach a stationary solution to the equations

\begin{align}
K &= -(R+B^TPB)^{-1}B^TPA \nonumber \\\\ 
P &= Q+A^TPA - K^TB^TPA \nonumber
\end{align}

as $T\to\infty$.  This is the [Discrete Algebraic Ricatti Equations (DARE)](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation) and $\lim_{T\to\infty}V_0(x)$ and $\lim_{T\to\infty}K_0$ are the cost-to-go and optimal control gain respectively for the [infinite horizon LQR problem](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Infinite-horizon,_discrete-time_LQR).  Indeed, one way to calculate the solution to the DARE is to iterate on the dynamic Ricatti equation.

## Implementation using GTSAM
You can view an example Jupyter notebook on [google colab](https://colab.research.google.com/drive/1pIUC6fQVMEaQ7QfJk8BvD0F60gShj3F4#sandboxMode=true){:target="_blank"} or
<a href="/assets/code_samples/lqr_control.zip" download>download</a> the modules/examples
that you can use in your
projects to:
* Calculate the closed loop gain matrix, K, using GTSAM
* Calculate the "cost-to-go" matrix, P (which is equivalent to the solutions to
  the dynamic Ricatti equation), using GTSAM
* Calculate the LQR solution for a non-zero, non-constant goal position, using GTSAM
* Visualize the cost-to-go and how it relates to factor graphs and the Ricatti
  equation
* and more!

A brief example of the open-loop finite horizon LQR problem using
factor graphs is shown below:

<div markdown="1" class="scrollablecontent" style="overflow: auto; height:600px;">

```python
def solve_lqr(A, B, Q, R, X0=np.array([0., 0.]), num_time_steps=500):
    '''Solves a discrete, finite horizon LQR problem given system dynamics in
    state space representation.
    Arguments:
        A, B: nxn state transition matrix and nxp control input matrix
        Q, R: nxn state cost matrix and pxp control cost matrix
        X0: initial state (n-vector)
        num_time_steps: number of time steps, T
    Returns:
        x_sol, u_sol: Txn array of states and Txp array of controls
    '''
    n = np.size(A, 0)
    p = np.size(B, 1)

    # noise models
    prior_noise = gtsam.noiseModel_Constrained.All(n)
    dynamics_noise = gtsam.noiseModel_Constrained.All(n)
    q_noise = gtsam.dynamic_cast_noiseModel_Diagonal_noiseModel_Gaussian(
        gtsam.noiseModel_Gaussian.Information(Q))
    r_noise = gtsam.dynamic_cast_noiseModel_Diagonal_noiseModel_Gaussian(
        gtsam.noiseModel_Gaussian.Information(R))
    # note: GTSAM 4.0.2 python wrapper doesn't have 'Information'
    # wrapper, use this instead if you are not on develop branch:
    #   `gtsam.noiseModel_Gaussian.SqrtInformation(np.sqrt(Q)))`

    # Create an empty Gaussian factor graph
    graph = gtsam.GaussianFactorGraph()

    # Create the keys corresponding to unknown variables in the factor graph
    X = []
    U = []
    for i in range(num_time_steps):
        X.append(gtsam.symbol(ord('x'), i))
        U.append(gtsam.symbol(ord('u'), i))

    # set initial state as prior
    graph.add(X[0], np.eye(n), X0, prior_noise)

    # Add dynamics constraint as ternary factor
    #   A.x1 + B.u1 - I.x2 = 0
    for i in range(num_time_steps-1):
        graph.add(X[i], A, U[i], B, X[i+1], -np.eye(n),
                  np.zeros((n)), dynamics_noise)

    # Add cost functions as unary factors
    for x in X:
        graph.add(x, np.eye(n), np.zeros(n), q_noise)
    for u in U:
        graph.add(u, np.eye(p), np.zeros(p), r_noise)

    # Solve
    result = graph.optimize()
    x_sol = np.zeros((num_time_steps, n))
    u_sol = np.zeros((num_time_steps, p))
    for i in range(num_time_steps):
        x_sol[i, :] = result.at(X[i])
        u_sol[i] = result.at(U[i])
    
    return x_sol, u_sol
```
</div>

## Future Work
The factor graph [(Figure 1)](#fg_scratch) for our finite horizon discrete LQR problem can be readily extended to LQG, iLQR, DDP, and reinforcement
learning using non-deterministic dynamics factors, nonlinear factors, discrete factor graphs, and other features of GTSAM (stay tuned for future posts).

<br />
<hr />
<br />

<!-- ********************************** APPENDIX ********************************** -->
## Appendix

<!-- ### Marginalization Cost on $x_1$
By substituting \eqref{eq:control_law} into \eqref{eq:potential_simplified}, we have the updated
potential function as a function of only $x_1$:
\\[ \begin{aligned} 
    \phi_1(x_1) &= x_1^T Q x_1 + (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + BKx_1) \\\\ 
    &= x_1^T(Q+ K_1^TRK_1 + A^TQA + K_1^TB^TQB - K_1^TB^TQA - A^TQBK_1)x_1  \\\\ 
    &= x_1^T[Q + A^TQA + K_1^T(R+B^TQB)K_1 - K_1^TB^TQA - A^TQBK_1]x_1 \\\\ 
    &= x_1^T(Q + A^TQA + A^TQBK_1 - K_1^TB^TQA - A^TQBK_1)x_1 \\\\ 
    &= x_1^T(Q + A^TQA - K_1^TB^TQA)x_1 
\end{aligned} \\] -->

### Least Squares Implementation in GTSAM
GTSAM can be specified to use either of two methods for solving the least squares problems that
appear in eliminating factor graphs: Cholesky Factorization or QR Factorization.  Both arrive at the same result, but we will take a look at QR since it more immediately illustrates the elimination algorithm at work.

<!-- plain table for formatting purposes -->
<style>
table, caption, tbody, tfoot, thead, table tr, table th, table tr:nth-child(even), td {
    margin: 0;
    padding: 0;
    border: 0;
    outline: 0;
    font-size: 100%;
    font-weight: normal;
    vertical-align: baseline;
    background: transparent;
    background-color: transparent;
}
table th {
    padding: 3px;
}
</style>

#### QR Factorization

<!-- ************************ QR block elimination ************************ -->
<!-- Slideshow container, based on https://www.w3schools.com/howto/howto_js_slideshow.asp -->
<a name="fig:qr_elim"></a>
<div class="slideshow-container" style="min-height:3in;">
  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \left[ \begin{array}{c} 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{-A      | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I 
    \end{array} \right]
    &
    \left[ \begin{array}{ccccc|c} 
        Q^{1/2} &   &       &       &       & 0\\\\ 
        I & -B      & -A    &       &       & 0\\\\ 
          & R^{1/2} &       &       &       & 0\\\\ 
          &         & Q^{1/2}&      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide0.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6a</b> Initial factor graph and elimination matrix</figcaption>
      </figure>
  </div>
  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I
    \end{bmatrix} & 
    \left[ \begin{array}{ccccc|c} 
        \color{red} {Q^{1/2}} &   &       &       &       & 0\\\\ 
        \color{red} I & \color{red} {-B}      & \color{red} {-A}    &       &       & 0\\\\ 
          & R^{1/2} &       &       &       & 0\\\\ 
          &         & Q^{1/2}&      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide1.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6b</b> Eliminate $x_2$: the two factors to replace are highlighted in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{c:cccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
        \hdashline 
          & \color{blue} {Q^{1/2}B} & \color{blue} {Q^{1/2}A} &      &       & 0\\\\ 
          & R^{1/2} &       &       &       & 0\\\\ 
          &                     & Q^{1/2}&      &       & 0\\\\ 
          &                     & I     & -B    & -A    & 0\\\\ 
          &                     &       & R^{1/2}&      & 0\\\\ 
          &                     &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide2.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6c</b> Eliminated $x_2$: the resulting binary cost factor is highlighted in blue</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{c:cccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
        \hdashline 
          & \color{red} {Q^{1/2}B} & \color{red} {Q^{1/2}A} &      &       & 0\\\\ 
          & \color{red} {R^{1/2}} &       &       &       & 0\\\\ 
          &                     & {Q^{1/2}} &      &       & 0\\\\ 
          &                     & I     & -B    & -A    & 0\\\\ 
          &                     &       & R^{1/2}&      & 0\\\\ 
          &                     &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide3.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6d</b> Eliminate $u_1$: the two factors to replace are highlighted in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2} | } I\\\\ 
        \vphantom{(P_1-Q)^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cc:ccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          \hdashline 
          &         & \color{blue} {(P_1-Q)^{1/2}} &      &       & 0\\\\ 
          &         & Q^{1/2} &      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide4.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6e</b> Eliminated $u_1$: the resulting unary cost factor on $x_1$ is shown in blue</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2} | } I\\\\ 
        \vphantom{(P_1-Q)^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{R^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cc:ccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          \hdashline 
          &         & \color{red} {(P_1-Q)^{1/2}} &      &       & 0\\\\ 
          &         & \color{red} {Q^{1/2}} &      &       & 0\\\\ 
          &         & \color{red} I     & \color{red} {-B}    & \color{red} {-A}    & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide6.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6f</b> Eliminate $x_1$: the three factors to replace are highlighted in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2}K_1 | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_0^{1/2} | } I\\\\ 
        \vphantom{P^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{ccc:cc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1&      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          \hdashline 
          &         &       &\color{blue} {P_1^{1/2}B} & \color{blue} {P_1^{1/2}A} & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide7.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6g</b> Eliminated $x_1$: the resulting binary cost factor is shown in blue</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2}K_1 | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_0^{1/2} | } I\\\\ 
        \vphantom{P^{1/2} | } I\\\\ 
        \vphantom{Q^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{ccc:cc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1&      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          \hdashline 
          &         &       &\color{red} {P_1^{1/2}B} &\color{red} {P_1^{1/2}A} & 0\\\\ 
          &         &       &\color{red} {R^{1/2}} &      & 0\\\\ 
          &         &       &       & Q^{1/2} & 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide8.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6h</b> Eliminate $u_0$: the two cost factors to replace are shown in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2}K_1 | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_0^{1/2} | } I\\\\ 
        \vphantom{(P_0-Q)^{1/2} | } I\\\\ 
        \vphantom{P^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cccc:c|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & D_0^{1/2}  & -D_0^{1/2}K_0 & 0\\\\ 
          \hdashline 
          &         &   &      & \color{blue} {(P_0-Q)^{1/2}} & 0\\\\ 
          &         &   &      & Q^{1/2}                    & 0\\\\ 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide9.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6i</b> Eliminated $u_0$: the resulting unary cost factor on $x_0$ is shown in blue.</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2}K_1 | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_0^{1/2} | } I\\\\ 
        \vphantom{(P_0-Q)^{1/2} | } I\\\\ 
        \vphantom{P^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cccc:c|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & D_0^{1/2}  & -D_0^{1/2}K_0 & 0\\\\ 
          \hdashline 
          &         &   &      & \color{red} {(P_0-Q)^{1/2}} & 0\\\\ 
          &         &   &      & \color{red} {Q^{1/2}}                    & 0\\\\ 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide10.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6j</b> Eliminate $x_0$: the final two factors to eliminate are shown in red.</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_1^{1/2}K_1 | } I\\\\ 
        \vphantom{I       | } 0\\\\ 
        \vphantom{D_0^{1/2} | } I\\\\ 
        \vphantom{P_0^{1/2} | } I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cccc:c|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & D_0^{1/2}  & -D_0^{1/2}K_0 & 0\\\\ 
          \hdashline 
          &         &   &      & \color{blue} {P_0^{1/2}} & 0\\\\ 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/Elimination/cropped_Slide11.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 6k</b> Final result: after eliminating $x_0$, the elimination matrix is upper-triangular and we can read off the control laws.</figcaption>
      </figure>
  </div>
  
  <!-- Next and previous buttons -->
  <a class="prev" onclick="plusSlides(-1,1)">&#10094;</a>
  <a class="next" onclick="plusSlides(1,1)">&#10095;</a>

</div> <!-- slideshow-container -->

<!-- The dots/circles -->
<div style="text-align:center">
  <span class="dot 1" onclick="currentSlide(1,1)"></span>
  <span class="dot 1" onclick="currentSlide(2,1)"></span>
  <span class="dot 1" onclick="currentSlide(3,1)"></span>
  <span class="dot 1" onclick="currentSlide(4,1)"></span>
  <span class="dot 1" onclick="currentSlide(5,1)"></span>
  <span class="dot 1" onclick="currentSlide(6,1)"></span>
  <span class="dot 1" onclick="currentSlide(7,1)"></span>
  <span class="dot 1" onclick="currentSlide(8,1)"></span>
  <span class="dot 1" onclick="currentSlide(9,1)"></span>
  <span class="dot 1" onclick="currentSlide(10,1)"></span>
  <span class="dot 1" onclick="currentSlide(11,1)"></span>
</div>
<br />
<div style="overflow: auto">
<table style="width:6.1in; margin: 0 auto;">
    <tr>
        <th>where</th><th>$P_{t}$</th><th>$=$</th><th>$Q + A^TP_{t+1}A - K_t^TB^TP_{t+1}A$</th><th>($P_2=Q$)</th>
    </tr><tr>
        <th></th><th>$D_{t}$</th><th>$=$</th><th>$R + B^TP_{t+1}B$</th>
    </tr><tr>
        <th></th><th>$K_t$</th><th>$=$</th><th>$-D_{t}^{-1/2}(R + B^TP_{t+1}B)^{-T/2}B^TP_{t+1}A$</th>
    </tr><tr>
        <th></th><th></th><th>$=$</th><th>$-(R + B^TP_{t+1}B)^{-1}B^TP_{t+1}A$</th>
    </tr>
</table>
</div>
<!-- ************************ end QR block elimination ************************ -->
  
<br />

The factorization process is illustrated in [Figure 6](#fig:qr_elim) for a 3-time step factor graph, where the noise matrices and elimination
matrices are shown with the corresponding states of the graph.  The noise matrix (NM) is $0$ for a
hard constraint and $I$ for a minimization objective.  The elimination matrix is formatted as an
augmented matrix $[A|b]$ for the linear least squares problem $\argmin\limits_x\|\|Ax-b\|\|_2^2$
with ${x=[x_2;u_1;x_1;u_0;x_0]}$ is the vertical concatenation of all state and control vectors.
The recursive expressions for $P$, $D$, and $K$ when eliminating control variables (i.e. $u_1$ in [Figure 6e](#fig:qr_elim){:onclick="currentSlide(5,1)"}) are derived from block QR Factorization.

Note that all $b_i=0$ in the augmented matrix for the LQR problem of finding minimal control to
reach state $0$, but simply changing values of $b_i$ intuitively extends GTSAM to solve
LQR problems whose objectives are to reach different states or even follow trajectories.

<!-- ### Final Symbolic Expressions of Factor Graph Evaluation
In the above solution, we have
\\[ \begin{aligned} 
K_1 &= -(R+B^TQB)^{-1}B^TQA\\\\ 
P_1 &= Q+A^TQA + A^TQBK_1\\\\ 
K_0 &= -(R+B^TV_1B)^{-1}B^TV_1A\\\\ 
P_0 &= Q + A^T V_1 A + A^T V_1 B K_0
\end{aligned} \\]

In general, the above factor graph and solution method can be expanded for an arbitrary number of time steps, $T$, arising in the iterative equations
\\[ \begin{aligned} 
    V_T &= Q \\\\ 
    K_t &= -( R + B^T V_{t+1} B )^{-1} B^T V_{t+1} A \\\\ 
    P_t &= Q + A^T V_{t+1} A + A^T V_{t+1} B K_t 
\end{aligned} \\]
and
\\[ \begin{aligned} 
    u_t &= K_t x_t
\end{aligned} \\]
which match the traditional algorithm using the Ricatti Equation for solving the finite-horizon discrete-time LQR problem.  As the number
of time steps grows, the solution for $V_0$ approaches the stationary solution to the algebraic
Ricatti equation and the solution for $K_0$ approaches the solution to the infinite-horizon
discrete-time LQR problem. -->

<!-- **************** JAVASCRIPT FOR SLIDESHOWS **************** -->
<script>
    var slideIndex = [1,1];
    showSlides(slideIndex[0], 0);
    showSlides(slideIndex[1], 1);

    // Next/previous controls
    function plusSlides(n, which) {
        showSlides(slideIndex[which] += n, which);
    }

    // Thumbnail image controls
    function currentSlide(n, which) {
        showSlides(slideIndex[which] = n, which);
    }

    // change image/slide
    function showSlides(n, which, triggeredByScroll) {
        var i;
        var slides = document.getElementsByClassName("mySlides "+which);
        var dots = document.getElementsByClassName("dot "+which);
        if (n > slides.length) {slideIndex[which] = 1}
        if (n < 1) {slideIndex[which] = slides.length}
        for (i = 0; i < slides.length; i++) {
            slides[i].style.display = "none";
        }
        for (i = 0; i < dots.length; i++) {
            dots[i].className = dots[i].className.replace(" active", "");
        }
        slides[slideIndex[which]-1].style.display = "block";
        dots[slideIndex[which]-1].className += " active";

        if (which==1){
            return
        }

        // when image changes, also scroll to the correct subsection in "Variable Elimination"
        var scrollable = document.getElementById("sec:elim_scrollable");
        var scrollLoc_state = document.getElementById("sec:elim_state").offsetTop - scrollable.offsetTop;
        var scrollLoc_ctrl = document.getElementById("sec:elim_ctrl" ).offsetTop - scrollable.offsetTop;
        var scrollLoc_bayes = document.getElementById("sec:elim_bayes").offsetTop - scrollable.offsetTop;
        var scroll_cur = scrollable.scrollTop;
        var scrollLoc;
        var div_state = document.getElementById("sec:elim_state_div");
        var div_ctrl = document.getElementById("sec:elim_ctrl_div");
        var div_bayes = document.getElementById("sec:elim_bayes_div");
        switch(slideIndex[which]) {
            case 1:
            case 2:
                div_state.style.display = "block";
                div_ctrl.style.display = "none";
                div_bayes.style.display = "none";
                // fadeIn(div_state);
                // fadeOut(div_ctrl, div_state);
                // fadeOut(div_bayes, div_state);
                return;
            case 3:
            case 4:
            case 5:
                div_state.style.display = "none";
                div_ctrl.style.display = "block";
                div_bayes.style.display = "none";
                // fadeOut(div_state, div_ctrl);
                // div_state.classList.toggle('hide');
                // fadeIn(div_ctrl);
                // fadeOut(div_bayes, div_ctrl);
                return;
            case 6:
            case 7:
            case 8:
            case 9:
                div_state.style.display = "none";
                div_ctrl.style.display = "none";
                div_bayes.style.display = "block";
                // fadeOut(div_state, div_bayes);
                // fadeOut(div_ctrl, div_bayes);
                // fadeIn(div_bayes);
                return;
        }
    }

    // // when scrolling through subsections in "Variable Elimination", also change the image to correspond
    // document.getElementById("sec:elim_scrollable").addEventListener("scroll", function (event) {
    //     var scrollable = document.getElementById("sec:elim_scrollable");
    //     var scrollLoc_state = document.getElementById("sec:elim_state").offsetTop - scrollable.offsetTop;
    //     var scrollLoc_ctrl = document.getElementById("sec:elim_ctrl" ).offsetTop - scrollable.offsetTop;
    //     // var scrollLoc_value = document.getElementById("sec:elim_value").offsetTop - scrollable.offsetTop;
    //     var scrollLoc_bayes = document.getElementById("sec:elim_bayes").offsetTop - scrollable.offsetTop;
        
    //     var scroll = this.scrollTop;
    //     if (scroll < scrollLoc_ctrl) {
    //         if (slideIndex[0] > 2) {showSlides(slideIndex[0]=1, 0, true)}
    //     }
    //     // else if (scroll < scrollLoc_value) {
    //     //     if ((slideIndex[0] < 3) || (slideIndex[0] > 4)) {showSlides(slideIndex[0]=3, 0, true)}
    //     // }
    //     else if ((scroll < scrollLoc_bayes) && (scroll < (scrollable.scrollHeight - scrollable.offsetHeight))) {
    //         if ((slideIndex[0] < 3) || (slideIndex[0] > 3)) {showSlides(slideIndex[0]=3, 0, true)}
    //     }
    //     else {
    //         if ((slideIndex[0] < 6)) {showSlides(slideIndex[0]=6, 0, true)}
    //     }
    // });

    function fadeOut(element, nextElement) {
        if (element.style.display == "none") {
            return;
        }
        element.addEventListener('webkitTransitionEnd', function () {
            element.style.display = "none";
            nextElement.style.display = "block";
        }, {once: true});
        element.addEventListener('mozTransitionEnd', function () {
            element.style.display = "none";
            nextElement.style.display = "block";
        }, {once: true});
        element.addEventListener('oTransitionEnd', function () {
            element.style.display = "none";
            nextElement.style.display = "block";
        }, {once: true});
        element.addEventListener('transitionend', function () {
            element.style.display = "none";
            nextElement.style.display = "block";
        }, {once: true});
        element.style.webkitTransitionDuration = "0.1s";
        element.style.mozTransitionDuration = "0.1s";
        element.style.oTransitionDuration = "0.1s";
        element.style.transitionDuration = "0.1s";
        element.style.opacity = "0";
    }
    function fadeIn(element) {
        if (element.style.display == "block") {
            return;
        }
        element.addEventListener('webkitTransitionEnd', function () {
        }, {once: true});
        element.addEventListener('mozTransitionEnd', function () {
        }, {once: true});
        element.addEventListener('oTransitionEnd', function () {
        }, {once: true});
        element.addEventListener('transitionend', function () {
        }, {once: true});
        element.style.webkitTransitionDuration = "0.1s";
        element.style.mozTransitionDuration = "0.1s";
        element.style.oTransitionDuration = "0.1s";
        element.style.transitionDuration = "0.1s";
        element.style.webkitTransitionDelay = "0.1s";
        element.style.mozTransitionDelay = "0.1s";
        element.style.oTransitionDelay = "0.1s";
        element.style.transitionDelay = "0.1s";
        element.style.opacity = "1";
    }
</script>