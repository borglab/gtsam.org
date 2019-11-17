---
layout: gtsam-post
title:  "LQR Control Using Factor Graphs"
---

<link rel="stylesheet" href="/assets/css/slideshow.css">

Authors: [Gerry Chen](https://gerry-chen.com) and [Yetong
Zhang](https://www.linkedin.com/in/yetong-zhang-9b810a105/)  

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

## Introduction
<a name="LQR_example"></a>
<figure class="center" style="width:90%;margin-bottom:">
  <a href="/assets/images/lqr_control/LQR_FGvsRicatti.png"><img src="/assets/images/lqr_control/LQR_FGvsRicatti.png"
    alt="Comparison between LQR control as solved by factor graphs and by the Ricatti Equation. (they are the same)" style="margin-bottom:10px;"/></a>
  <figcaption><b>Figure 1</b> Example LQR control solutions as solved by factor graphs (middle) and the traditional Discrete Algebraic Ricatti Equations (right).  The optimal control gains and cost-to-go factors are compared (left).  All plots show exact agreement between factor graph and Ricatti equation solutions.</figcaption>
</figure>
<br />

In this post we explain how optimal control problems can be formulated as factor graphs and solved
by performing variable elimination on the factor graph.

Specifically, we will show the factor graph formulation and solution for the
**Linear Quadratic Regulator (LQR)** as this problem conveys the essential ideas of using factor
graphs for optimal control, though they can be readily extended to LQG, iLQR, DDP, and reinforcement
learning (stay tuned for future posts).  We consider the **finite-horizon, discrete LQR problem**
(though the control law converges to the infinite-horizon case quite quickly as illustrated in
[Figure 1a](#LQR_example)).  The task is to find the optimal controls $u_k$ at time instances $t_k$
so that a total cost is minimized, given \eqref{eq:dyn_model} a dynamics model,
\eqref{eq:state_cost} a cost function on states, and \eqref{eq:action_cost} a cost
function on actions. In the linear-quadratic problem, these are of the
form ([Wikipedia](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Finite-horizon,_discrete-time_LQR)):

\\[ \begin{equation} x_{k+1} = Ax_k + Bu_k \label{eq:dyn_model} \end{equation} \\]
\\[ \begin{equation} L_x(x_k) = x_k^T Q x_k \label{eq:state_cost} \end{equation} \\]
\\[ \begin{equation} L_u(u_k) = u_k^T R u_k \label{eq:action_cost} \end{equation} \\]

The optimal controls over time can be expressed as the constrained optimization problem:

\\[ \argmin\limits_{u_{1\sim k}}\sum\limits_{i=1}^n x_i^T Q x_i + u_i^T R u_i \\]
\\[ s.t. ~~ x_{t+1}=Ax_t+Bu_t ~~\text{for } t=1 \text{ to } T-1 \\]

We can visualize the objective function and constraints in the form of a factor
graph as shown in [Figure 2](#fg_scratch). This is a simple Markov chain, with the oldest
states and actions on the left, and the newest states and actions on the right. **The
ternary factors represent the dynamics model constraints and the unary
factors represent the state and control costs we seek to minimize via least-squares.**

<a name="fg_scratch"></a>
<figure class="center">
  <img src="/assets/images/lqr_control/VE/fg_lqr.png"
    alt="Factor graph structure. The objective factors are marked with dashed lines, and the constrain factors are marked with solid lines." />
    <figcaption><b>Figure 2</b> Factor graph structure for an LQR problem with 3 time steps. The cost factors are marked with dashed lines and the dynamics constraint factors are marked with solid lines.</figcaption>
</figure>

## Variable Elimination
To optimize the factor graph, which represents minimizing the least squares objectives above, we can simply eliminate the factors from right
to left.  In this section we demonstrate the variable elimination algebraically, but discussion on the implementation in GTSAM is also
provided in the [Appendix](#least-squares-implementation-in-gtsam).

<!-- ********************** BEGIN VARIABLE ELIMINATION SLIDESHOW ********************** -->
<!-- Slideshow container, based on https://www.w3schools.com/howto/howto_js_slideshow.asp -->
<div class="slideshow-container">
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">2 / 3</div> -->
    <a name="fig_eliminate_x"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg1.png"
        alt="Elimination of state $x_2$" />
        <figcaption><b>Figure 3a</b> Elimination of state $x_2$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">2 / 3</div> -->
    <a name="fig_eliminate_x"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg2.png"
        alt="Elimination of state $x_2$" />
        <figcaption><b>Figure 3b</b> Elimination of state $x_2$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_eliminate_u"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg3.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 4a</b> Elimination of state $u_1$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_eliminate_u"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg4.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 4b</b> Elimination of state $u_1$</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg5.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5a</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg6.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5b</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg7.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5c</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides 0" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg8.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5d</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
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
</style>
<!-- ********************** END VARIABLE ELIMINATION SLIDESHOW ********************** -->

<!-- ************************ BEGIN SCROLLABLE ELIMINATION DESCRIPTION ************************ -->
<div class="scrollablecontent" markdown="1" id="sec:elim_scrollable"
    style="overflow-y: scroll; height:400px; overflow-x: hidden; background-color:rgba(0,0,0,0.05); padding:0 8px; margin-bottom: 10px;">
<!-- ************** STATE ************** -->
<a id="sec:elim_state"></a>
### Eliminate a State
Let us start at the last state, $x_2$. Gathering the two factors (marked in
red [Figure 3a](#fig_eliminate_x)), we have \eqref{eq:potential} the objective function, $\phi_1$, and \eqref{eq:constrain} the constraint equation on $x_2$, $u_1$ and $x_1$:

\begin{equation} \phi_1(x_2) = x_2^T Q x_2 \label{eq:potential} \end{equation}

\begin{equation} x_2 = Ax_1 + Bu_1 \label{eq:constrain} \end{equation}

By substituting $x_2$ from the dynamics constraint \eqref{eq:constrain} into the objective function
\eqref{eq:potential}, we create a new factor representing
the cost of state $x_2$ as a function of $x_1$ and $u_1$:

\begin{equation} \phi_2(x_1, u_1) = (Ax_1 + Bu_1)^T Q (Ax_1 + Bu_1)
\label{eq:potential_simplified} \end{equation}

The resulting factor graph is illustrated in [Figures 3a](#fig_eliminate_x) and
[3b](#fig_eliminate_x). To summarize, we used the dynamics constraint to eliminate variable
$x_2$ as well as the two factors marked in red, and replaced them with a new binary cost factor on $x_1$
and $u_1$, marked in blue.
<!-- ************** CONTROL ************** -->
<a id="sec:elim_ctrl"></a>
### Eliminate a Control
<!-- Now \eqref{eq:potential_simplified} defines an (unnormalized) joint
Gaussian density on variables $x_1$ and $u_1$.  -->
To eliminate $u_1$, we seek to replace the two factors marked red in [Figure 4a](#fig_eliminate_u)
with a new cost factor on $x_1$ and an equation for the optimal control $$u_1^*(x_1)$$.

Adding the control cost \eqref{eq:action_cost} to \eqref{eq:potential_simplified}, the combined cost of the
two red factors in [Figure 4a](#fig_eliminate_u) is given by:

\begin{equation} \phi_3(x_1, u_1) = u_1^TRu_1 + (Ax_1 + Bu_1)^T Q (Ax_1 + Bu_1)
\label{eq:potential_u1} \end{equation}

We minimize $\phi_3$ by
setting the derivative of \eqref{eq:potential_u1} wrt $u_1$ to zero
<!-- (detailed calculation in the [Appendix](#eliminate-u_1)),  -->
yielding the expression for the optimal control input $u_1^*$ as 

\\[ \begin{align} 
u_1^*(x_1) &= -(R+B^TQB)^{-1}B^TQAx_1 \label{eq:control_law} \\\\ 
&= K_1x_1 \nonumber
\end{align} \\]

where $K_1\coloneqq -(R+B^TQB)^{-1}B^TQA$.

Finally, we substitute the expression of our optimal control, $$u_1^*$$,
into our potential \eqref{eq:potential_u1}
<!-- (detailed calculation in the [Appendix](#marginalization-cost-on-x_1)) -->
to obtain a new unary cost factor on $x_1$:

\begin{align}
    \phi_4(x_1) &= \phi_3(x_1, u_1^*(x_1)) \nonumber \\\\ 
        &= (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + BKx_1) \label{eq:potential_x1}
\end{align}

### Combine Unary Cost Factors
We add the existing unary state cost factor on $x_1$ to the acculumated cost
\eqref{eq:potential_x1} to obtain the cost-to-go, 

\\[ \begin{align} 
V_1(x_1) &= x_1^T Q x_1 + (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + 
BKx_1) \nonumber \\\\ 
&= x_1^T(Q+A^TQA - K_1^TB^TQA)x_1 \label{eq:cost_update} \\\\ 
&= x_1^T V_1 x_2 \nonumber
\end{align} \\]
where $V_1\coloneqq Q+A^TQA - K_1^TB^TQA$.

Note that we simplified $K_1^TRK_1 + K_1^TB^TQBK_1 = K_1^TB^TQA$ by substituting in the non-transposed $K_1$ using
\eqref{eq:control_law}.

As illustrated in [Figure 4](#fig_eliminate_u), through the above steps, we can eliminate variable
$x_2$, $u_2$ as well as three factors marked in red, and replace them with a new factor on $x_1$
marked in blue, with potential $x_1^TV_1x_1$ , which represents the marginalized cost on state
$x_1$.
<!-- ************** BAYES NET ************** -->
<a id="sec:elim_bayes"></a>
### Turning into a Bayes Network
By eliminating all the variables from right to left, we can get a Bayes network
as shown in [Figure 5d](#fig_bayes_net). Everytime we eliminate an older state
and control, we simply repeat the steps in [Eliminate a state](#eliminate-a-state) and [Eliminate a control](#eliminate-a-control): we express the
older state $x_{k+1}$ with the dynamics model, and express the control $u_k$ as
a function of state $x_k$, then generate a new factor on $x_k$ representing the
"cost-to-go" function $x_k^TV_kx_k$.

Eliminating a general state, $x_{k+1}$, and control $u_k$, we obtain the recurrence relations:

\begin{equation} \boxed{K_k = -(R+B^TV_{k+1}B)^{-1}B^TV_{k+1}A} \label{eq:control_update_k} \end{equation}

\begin{equation} \boxed{V_k = Q+A^TV_{k+1}A - K_k^TB^TV_{k+1}A} \label{eq:cost_update_k} \end{equation}

with $V_{T}=Q$ is the cost at the last time step.
</div> <!-- scrollablecontent -->
<!-- ************************ END SCROLLABLE ELIMINATION DESCRIPTION ************************ -->

## Equivalence to the Ricatti Equation
In [Wikipedia](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Finite-horizon,_discrete-time_LQR), the control law and cost function for LQR are given by

\\[ u_k = K_kx_k \\]

\begin{equation} K_k = -(R+B^TP_{k+1}B)^{-1}B^TP_{k+1}A \label{eq:control_update_k_ricatti} \end{equation}

\begin{equation} P_k = Q+A^TP_{k+1}A - K_k^TB^TP_{k+1}A \label{eq:cost_update_k_ricatti} \end{equation}

with $P_k$ commonly referred to as the solution to the dynamic Ricatti equation and $P_T=Q$ is the
value of the Ricatti function at the final time step.

Note that \eqref{eq:control_update_k_ricatti} and \eqref{eq:cost_update_k_ricatti} correspond to
the same results as we derived in \eqref{eq:control_update_k} and \eqref{eq:cost_update_k}
respectively, where the Ricatti solutions, $P_k$, correspond to $V_k$ in our factor graph elimination.

## Intuition
In our factor graph representation, it is becomes obvious that the dynamic Ricatti equation
solutions, $P_k$, correspond to the total cost at the state $x_k$ that will be accrued for the remainder
of the trajectory and control assuming optimal control after $x_k$.  Referred to as "cost-to-go",
"return cost", and "value function" by different literatures, we denote this cost as $\phi_k$, given by 
\\[ \phi_k = x_k^TP_kx_k \\]
This "cost-to-go" is depicted as a heatmap in [Figure 1](#LQR_example).  From
\eqref{eq:control_update_k}, the optimal control, $K_k$,
represents a balance between achieving a small "cost-to-go" next time step ($B^TV_{k+1}B$) and exerting a small
amount of control this time step ($R$).

## Implementation using GTSAM
We provide some 
modules and examples available to
<a href="/assets/code_samples/lqr_control.zip" download>download</a>
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

<br />
<hr />
<br />

<!-- ********************************** APPENDIX ********************************** -->
## Appendix
### Eliminate $u_1$
Setting the derivative w.r.t. $u_1$ of the objective function, $\phi_1(x_1, u_1)$, in \eqref{eq:potential_simplified} to zero:
\\[ Ru_1 + B^TQ(Ax_1+Bu_1) = 0 \\]
we solve to obtain the result shown in \eqref{eq:control_law}:
\\[ \begin{aligned} 
    & (R+B^TQB)u_1 + B^TQAx_1 = 0 \\\\ 
    & u_1 = -(R+B^TQB)^{-1}B^TQAx_1 
\end{aligned} \\]

### Marginalization Cost on $x_1$
By substituting \eqref{eq:control_law} into \eqref{eq:potential_simplified}, we have the updated
potential function as a function of only $x_1$:
\\[ \begin{aligned} 
    \phi_1(x_1) &= x_1^T Q x_1 + (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + BKx_1) \\\\ 
    &= x_1^T(Q+ K_1^TRK_1 + A^TQA + K_1^TB^TQB - K_1^TB^TQA - A^TQBK_1)x_1  \\\\ 
    &= x_1^T[Q + A^TQA + K_1^T(R+B^TQB)K_1 - K_1^TB^TQA - A^TQBK_1]x_1 \\\\ 
    &= x_1^T(Q + A^TQA + A^TQBK_1 - K_1^TB^TQA - A^TQBK_1)x_1 \\\\ 
    &= x_1^T(Q + A^TQA - K_1^TB^TQA)x_1 
\end{aligned} \\]

### Least Squares Implementation in GTSAM
GTSAM can be specified to use either of two methods for solving the least squares problems that
appear in eliminating factor graphs: Cholesky Factorization or QR Factorization.  We
will find that both arrive at the same solution matching the dynamic Ricatti equation solution.
#### Cholesky Factorization
We have established that the cost optimization in the LQR problem can be expressed as a least
squares problem $\argmin\limits_x\|\|Ax-b\|\|_2^2$ where ${x=[x_2;u_1;x_1;u_0;x_0]}$ is the vertical
concatenation of all state and control vectors, $A$ is a matrix representing quadratic costs, and
$b=0$ for the case of optimizing both control and state cost towards 0.

To solve the linear least squares problem $\argmin\limits_x\|\|Ax-b\|\|_2^2$, we can reformulate the
problem as the equivalent linear equation $A^TAx=A^Tb$. Cholesky factorization computes a factorization on $A^TA$ such that
$A^TA=R^TR$ thereby allowing solving by back-substitution.
Instead of considering the full $A^TA$ matrix, for demonstrative purposes we can consider just a
single *clique*, as it is implemented in GTSAM for computational reasons.  A clique is a set of
variables which are fully connected. In each clique, GTSAM first performs variable elimination with
the strict constraints to convert the constrained optimization problem into an unconstrained one.
Then, GTSAM performs Cholesky factorization on rest of the variables. For the clique circled in red
in [Figure 6](#fig:clique), we first eliminate $x_2$ using the dynamics constraint to obtain our least
squares clique formulation: ${x=[u_1;x_1]}$, $b=0$, and $A$ and $A^TA$ are given by

\\[ \begin{aligned} 
    A= 
    \begin{bmatrix} 
      Q^{\frac{1}{2}}B & Q^{\frac{1}{2}}A \\\\ 
      R^{\frac{1}{2}} & \\\\ 
      & Q^{\frac{1}{2}} 
    \end{bmatrix} 
\end{aligned} \\]
\\[ \begin{aligned} 
    A^TA= 
    \begin{bmatrix} 
      R + B^TQB & B^TQA \\\\ 
      A^TQB & Q+A^TQA 
    \end{bmatrix} 
\end{aligned} \\]
Applying block Cholesky decomposition on the matrix, we have
\\[ \begin{aligned} 
    R = 
    \begin{bmatrix} 
      D_1^{\frac{1}{2}} & -D_1^{\frac{1}{2}}K_1 \\\\ 
      0 & V_1^{\frac{1}{2}} 
    \end{bmatrix} 
\end{aligned} \\]

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
<!-- "where D = ..., K = ..., V = ..." -->
<div style="overflow: auto">
<table style="width:3.8in;">
    <tr>
        <th>where</th><th>$D_1$</th><th>$=$</th><th>$R + B^TQB$</th>
    </tr><tr>
        <th></th><th>$K_1$</th><th>$=$</th><th>$-(R + B^TQB)^{-1}B^TQA$</th>
    </tr><tr>
        <th></th><th>$V_1$</th><th>$=$</th><th>$Q + A^TQA - K_1^TD_1^{T/2}D_1^{1/2}K_1$</th>
    </tr><tr>
        <th></th><th></th><th>$=$</th><th>$Q + A^TQA - K_1^TB^TQA$</th>
    </tr>
</table>
</div>

which can be verified by multiplying out $R^TR$.  It is now evident that the top block row of $R$
gives the control law (i.e. $D_1^{1/2}u_1 - D_1^{1/2}K_1x_1 = 0$) and the bottom block row gives the
new "cost-to-go" for the next clique.

<a name="fig:clique"></a>
<figure class="center">
  <img src="/assets/images/lqr_control/VE/cliq1a.png"
    alt="Single clique in LQR factor graph." />
    <figcaption><b>Figure 6</b> A single clique in the LQR factor graph formulation is circled in red.</figcaption>
</figure>

#### QR Factorization
The process is illustrated in [Figure 7](#fig:qr_elim) where the noise matrices and elimination
matrices are shown with the corresponding states of the graph.  The noise matrix (NM) is $0$ for a
hard constraint and $I$ for a minimization objective.  The elimination matrix is formatted as an
augmented matrix $[A|b]$ for the linear least squares problem $\argmin\limits_x\|\|Ax-b\|\|_2^2$
with ${x=[x_2;u_1;x_1;u_0;x_0]}$ is the vertical concatenation of all state and control vectors.

<!-- ************************ QR block elimination ************************ -->
<!-- Slideshow container, based on https://www.w3schools.com/howto/howto_js_slideshow.asp -->
<a name="fig:qr_elim"></a>
<div class="slideshow-container" style="min-height:3in;">
  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I
    \end{bmatrix} & 
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
        <img src="/assets/images/lqr_control/elimination_steps/fg0.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7a</b> Initial factor graph and elimination matrix</figcaption>
      </figure>
  </div>
  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I
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
        <img src="/assets/images/lqr_control/elimination_steps/fg1.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7b</b> Eliminate $x_2$: the two factors to eliminate are highlighted in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
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
        <img src="/assets/images/lqr_control/elimination_steps/fg2.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7c</b> Eliminated $x_2$: the resulting binary cost factor is highlighted in blue</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{c:cccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
        \hdashline 
          & \color{red} {Q^{1/2}B} & \color{red} {Q^{1/2}A} &      &       & 0\\\\ 
          & \color{red} {R^{1/2}} &       &       &       & 0\\\\ 
          &                     & \color{red} {Q^{1/2}} &      &       & 0\\\\ 
          &                     & I     & -B    & -A    & 0\\\\ 
          &                     &       & R^{1/2}&      & 0\\\\ 
          &                     &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/elimination_steps/fg3.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7d</b> Eliminate $u_1$: the three factors to eliminate are highlighted in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cc:ccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          \hdashline 
          &         & \color{blue} {P_1^{1/2}} &      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/elimination_steps/fg4.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7e</b> Eliminated $u_1$: the resulting unary cost factor on $x_1$ is shown in blue</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cc:ccc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          \hdashline 
          &         & \color{red} {P_1^{1/2}} &      &       & 0\\\\ 
          &         & \color{red} I     & \color{red} {-B}    & \color{red} {-A}    & 0\\\\ 
          &         &       & R^{1/2}&      & 0\\\\ 
          &         &       &       & Q^{1/2}& 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/elimination_steps/fg5.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7f</b> Eliminate $x_1$: the two factors to eliminate are highlighted in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        I\\\\ 
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
        <img src="/assets/images/lqr_control/elimination_steps/fg6.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7g</b> Eliminated $x_1$: the resulting binary cost factor is shown in blue</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
        I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{ccc:cc|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1&      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          \hdashline 
          &         &       &\color{red} {P_1^{1/2}B} &\color{red} {P_1^{1/2}A} & 0\\\\ 
          &         &       &\color{red} {R^{1/2}} &      & 0\\\\ 
          &         &       &       &\color{red} {Q^{1/2}} & 0 
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/elimination_steps/fg7.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7h</b> Eliminate $u_0$: the three cost factors to combine are shown in red</figcaption>
      </figure>
  </div>

  <div class="mySlides 1" style="text-align: center;">
      <figure class="center" style="width:75%">
<div markdown="1" align="left" style="width:100%; height:2.6in; overflow:auto">
\\( \begin{array}{cc} 
    \text{NM} & \text{Elimination Matrix} \\\\ 
    \begin{bmatrix} 
        0\\\\ 
        I\\\\ 
        0\\\\ 
        I\\\\ 
        I\\\\ 
    \end{bmatrix} & 
    \left[ \begin{array}{cccc:c|c} 
        I & -B      & -A    &       &       & 0\\\\ 
          & D_1^{1/2} & -D_1^{1/2}K_1 &      &       & 0\\\\ 
          &         & I     & -B    & -A    & 0\\\\ 
          &         &       & D_0^{1/2}  & -D_0^{1/2}K_0 & 0\\\\ 
          \hdashline 
          &         &       &       & \color{blue} {P_0^{1/2}}   & 0
    \end{array} \right]
    \end{array} \\)
</div>
        <img src="/assets/images/lqr_control/elimination_steps/fg8.png" alt="factor graph partially eliminated" />
        <figcaption><b>Figure 7i</b> Final result: after eliminating $u_0$, the elimination matrix is upper-triangular and we can read off the control laws.</figcaption>
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

Note that all $b_i=0$ in the augmented matrix for the LQR problem of finding minimal control to
reach state $0$, but simply changing values of $b_i$ can intuitively extend GTSAM to solve
LQR problems whose objectives are to reach different states or even trajectories.

The recursive expressions for $P$ and $V$ are derived from block QR Factorization.
However, block QR factorization is non-trivial to follow so, for demonstrative purposes, we can also
find their forms algebraically using the "completing the square" technique.  Taking, for example,
the elimination of $u_0$ ,

\\[ \begin{aligned} \scriptstyle 
    J(u_0, x_0) & \scriptstyle=~ \|\| P_1^{1/2}Bu_0 ~+~ P_1^{1/2}Ax_0\|\|^2_2 ~+~ \|\|R^{1/2}u_0\|\|^2_2 \qquad+ \|\|Q^{1/2}x_0\|\|^2_2 \\\\ 
        & \scriptstyle=~ u_0^T(B^TP_1B+R)u_0 ~+~ 2u_0^TB^TP_1Ax_0 ~+~ x_0^TA^TP_1Ax_0 \qquad+ x^TQx \\\\ 
        & \scriptstyle=~ u_0^T(B^TP_1B+R)u_0 ~+~ 2u_0^T(B^TP_1B+R)^{1/2}(B^TP_1B+R)^{-1/2}B^TP_1Ax_0 ~+~ x_0^TA^TP_1Ax_0 \qquad+ x_0^TQx_0 \\\\ 
        & \scriptstyle=~ \|\|(B^TP_1B+R)^{1/2}u_0 ~+~ (B^TP_1B+R)^{-1/2}B^TP_1Ax_0\|\|^2_2 ~-~ x_0^T(A^TP_1^TB(B^TP_1B+R)^{-T}B^TP_1A)x_0 ~+~ x_0^TA^TP_1Ax_0 ~+~ x^TQx  \\\\ 
        & \scriptstyle=~ \|\|(B^TP_1B+R)^{1/2}u_0 ~+~ (B^TP_1B+R)^{-1/2}B^TP_1Ax_0\|\|_2^2 ~+~ \|\|(Q ~+~ A^TP_1A)^{1/2}x_0 ~-~ A^TP_1^TB(B^TP_1B+R)^{-T}B^TP_1A\|\|_2^2  \\\\ 
        & \scriptstyle=~ \|\|(B^TP_1B+R)^{1/2}u_0 ~+~ (B^TP_1B+R)^{-1/2}B^TP_1Ax_0\|\|_2^2 ~+~ \|\|(Q ~+~ A^TP_1A ~-~ K_0^TB^TP_1A)^{1/2}x_0\|\|_2^2 \\\\ 
        & \scriptstyle=~ \|\|V\_{0,0}u_0 ~+~ V\_{0,1}x_0\|\|_2^2 ~+~ \|\|P_0^{1/2}x_0\|\|_2^2 
\end{aligned} \\]

### Final Symbolic Expressions of Factor Graph Evaluation
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
discrete-time LQR problem.

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
        var scrollLoc_ctrl = document.getElementById("sec:elim_ctrl").offsetTop - scrollable.offsetTop;
        var scrollLoc_bayes = document.getElementById("sec:elim_bayes").offsetTop - scrollable.offsetTop;
        var scroll_cur = scrollable.scrollTop;
        var scrollLoc;
        switch(slideIndex[which]) {
            case 1:
            case 2:
                return; // never force scroll up, only force scroll down
                // scrollLoc = scrollLoc_state;
                // break;
            case 3:
            case 4:
                if (scroll_cur >= scrollLoc_ctrl) {return;}
                scrollLoc = scrollLoc_ctrl;
                break;
            case 5:
            case 6:
            case 7:
            case 8:
                if (scroll_cur >= scrollLoc_bayes) {return;}
                scrollLoc = scrollLoc_bayes;
                break;
        }
        if (typeof triggeredByScroll === 'undefined') {
            scrollable.scrollTo(0, scrollLoc);
        }
    }

    // when scrolling through subsections in "Variable Elimination", also change the image to correspond
    document.getElementById("sec:elim_scrollable").addEventListener("scroll", function (event) {
        var scrollable = document.getElementById("sec:elim_scrollable");
        var scrollLoc_state = document.getElementById("sec:elim_state").offsetTop - scrollable.offsetTop;
        var scrollLoc_ctrl = document.getElementById("sec:elim_ctrl").offsetTop - scrollable.offsetTop;
        var scrollLoc_bayes = document.getElementById("sec:elim_bayes").offsetTop - scrollable.offsetTop;
        
        var scroll = this.scrollTop;
        if (scroll < scrollLoc_ctrl) {
            if (slideIndex[0] > 2) {showSlides(slideIndex[0]=1, 0, true)}
        }
        else if ((scroll < scrollLoc_bayes) && (scroll < (scrollable.scrollHeight - scrollable.offsetHeight))) {
            if ((slideIndex[0] < 3) || (slideIndex[0] > 4)) {showSlides(slideIndex[0]=3, 0, true)}
        }
        else {
            if ((slideIndex[0] < 5)) {showSlides(slideIndex[0]=5, 0, true)}
        }
    });
</script>