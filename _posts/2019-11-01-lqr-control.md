---
layout: gtsam-post
title:  "LQR Control Using Factor Graphs"
---

<link rel="stylesheet" href="/assets/css/slideshow.css">

Authors: [Gerry Chen](https://gerry-chen.com) and [Yetong Zhang](https://www.linkedin.com/in/yetong-zhang-9b810a105/)  

## Introduction
<a name="LQR_example"></a>
<figure class="center" style="width:90%;margin-bottom:">
  <a href="/assets/images/lqr_control/LQR_FGvsRicatti.png"><img src="/assets/images/lqr_control/LQR_FGvsRicatti.png"
    alt="Comparison between LQR control as solved by factor graphs and by the Ricatti Equation. (they are the same)" style="margin-bottom:10px;"/></a>
  <figcaption><b>Figure 1</b> Example LQR control solutions as solved by factor graphs (middle) and the traditional Discrete Algebraic Ricatti Equations (right).  The optimal control gains and cost-to-go factors are compared (left).  All plots show exact agreement between factor graph and Ricatti equation solutions.</figcaption>
</figure>
<br />
In this post we explain how optimal control problems can be formulated as factor graphs and solved by performing variable elimination on the factor graph.

In this post, we will mainly be concerned with the Linear Quadratic Regulator
(LQR) as this problem conveys the essential ideas, though they can be
extended to LQG, iLQR, DDP, and reinforcement learning (stay tuned for future
posts).  We consider the finite-horizon, discrete LQR problem (though the control law converges to
the infinite-horizon case quite quickly as illustrated in [Figure 1a](#LQR_example)).  The task is
to find the optimal controls $u_k$ at time instances $t_k$ so that
a total cost is minimized, given [(1)](#eq:dyn_model) a dynamics model,
[(2)](#eq:state_cost) a cost function on states, and [(3)](#eq:action_cost) a cost
function on actions. In the linear-quadratic case we assume these are of the
form \cite{}:

<a name="eq:dyn_model"></a> \\[ x_{k+1} = Ax_k + Bu_k \tag{1} \\]
<a name="eq:state_cost"></a> \\[ L(x_k) = x_k^T Q x_k \tag{2} \\]
<a name="eq:action_cost"></a> \\[ L(u_k) = u_k^T R u_k \tag{3} \\]

The optimal controls over time can be obtained by minimizing the total cost:

\\[ \argmin\limits_{u_{1\sim k}}\sum\limits_{i=1}^n x_i^T Q x_i + u_i^T R u_i \\]
\\[ s.t. ~~ x_{t+1}=Ax_t+Bu_t ~~\text{for } t=1 \text{ to } T-1 \\]

We can visualize the objective function and constraints in the form of a factor
graph as shown in [Figure 2](#LQR_example). This is a simple Markov chain, with the oldest
states and actions on the left, and the newest states and actions on the right. The
ternary factors represent the dynamics model constraints and the unary
factors represent the state and control costs.

<a name="fg_scratch"></a>
<figure class="center">
  <img src="/assets/images/lqr_control/VE/fg_lqr.png"
    alt="Factor graph structure. The objective factors are marked with dashed lines, and the constrain factors are marked with solid lines." />
    <figcaption><b>Figure 2</b> Factor graph structure for an LQR problem with 3 time steps. The cost factors are marked with dashed lines and the dynamics constraint factors are marked with solid lines.</figcaption>
</figure>

## Variable Elimination
To optimize the factor graph, which represents minimizing the least square objectives above, we can simply eliminate the factors from right
to left.

<!-- Slideshow container, based on https://www.w3schools.com/howto/howto_js_slideshow.asp -->
<div class="slideshow-container">
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">2 / 3</div> -->
    <a name="fig_eliminate_x"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg1.png"
        alt="Elimination of state $x_2$" />
        <figcaption><b>Figure 3a</b> Elimination of state $x_2$</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">2 / 3</div> -->
    <a name="fig_eliminate_x"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg2.png"
        alt="Elimination of state $x_2$" />
        <figcaption><b>Figure 3b</b> Elimination of state $x_2$</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_eliminate_u"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg3.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 4a</b> Elimination of state $u_1$</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_eliminate_u"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg4.png"
        alt="Elimination of state $u_1$" />
        <figcaption><b>Figure 4b</b> Elimination of state $u_1$</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg5.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5a</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg6.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5b</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg7.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5c</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <div class="mySlides" style="text-align: center;">
    <!-- <div class="numbertext">3 / 3</div> -->
    <a name="fig_bayes_net"></a>
    <figure class="center">
    <img src="/assets/images/lqr_control/elimination_steps/fg8.png"
        alt="Bayes net" />
        <figcaption><b>Figure 5d</b> Repeat elimination until the graph is reduced to a Bayes net</figcaption>
    </figure>
  </div>
  <!-- Next and previous buttons -->
  <a class="prev" onclick="plusSlides(-1)">&#10094;</a>
  <a class="next" onclick="plusSlides(1)">&#10095;</a>

</div> <!-- slideshow-container -->

<!-- The dots/circles -->
<div style="text-align:center">
  <span class="dot" onclick="currentSlide(1)"></span>
  <span class="dot" onclick="currentSlide(2)"></span>
  <span class="dot" onclick="currentSlide(3)"></span>
  <span class="dot" onclick="currentSlide(4)"></span>
  <span class="dot" onclick="currentSlide(5)"></span>
  <span class="dot" onclick="currentSlide(6)"></span>
  <span class="dot" onclick="currentSlide(7)"></span>
  <span class="dot" onclick="currentSlide(8)"></span>
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

<!-- ************************ BEGIN SCROLLABLE ELIMINATION DESCRIPTION ************************ -->
<div class="scrollablecontent" markdown="1" id="sec:elim_scrollable"
    style="overflow-y: scroll; height:400px; overflow-x: hidden; background-color:rgba(0,0,0,0.05); padding:0 8px; margin-bottom: 10px;">
<!-- ************************ STATE ************************ -->
<a id="sec:elim_state"></a>
### Eliminate a State
Let us start at the last state, $x_2$. Gathering the three factors (marked in
red [Figure 3a](#fig_eliminate_x)), we have the following objective function, $\phi_1$, and constraint equation, [(4)](#eq:constrain), on $x_2$, $u_1$ and $x_1$:

<a name="eq:potential"></a>
\\[ \phi_1(x_1, u_1, x_2) = x_1^T Q x_1 + u_1^T R u_1 + x_2^T Q x_2
\tag{4} \\]
<a name="eq:constrain"></a>
\\[ x_2 = Ax_1 + Bu_1 \tag{5} \\]

By substituting $x_2$ into [(4)](#eq:potential) using the [(5)](#eq:constrain), we can rewrite
$\phi_1$ as a function of $x_1$ and $u_1$:

<a name="eq:potential_simplified"></a>
\\[ \phi_1(x_1, u_1) = x_1^T Q x_1 + u_1^T R u_1 + (Ax_1 + Bu_1)^T Q (Ax_1 + Bu_1)
\tag{6} \\]

The resulting factor graph is illustrated in [Figures 3a](#fig_eliminate_x) and
[3b](#fig_eliminate_x). To summarize, we used the dynamics constraint to eliminate variable
$x_2$ as well as the two factors marked in red, and replace them with a new binary factor on $x_1$
and $u_1$, marked in blue.
<!-- ************************ CONTROL ************************ -->
<a id="sec:elim_ctrl"></a>
### Eliminate a Control
Note that [(6)](#eq:potential_simplified) defines an (unnormalized) joint
Gaussian density on variables $x_1$ and $u_1$. We solve for the mean of $u_1$ by
setting the derivative of [(6)](#eq:potential_simplified) wrt $u_1$ to zero,
yielding the expression of $u_1$ given $x_1$ as (detailed calculation in the
[Appendix](#eliminate-u_1))

<a name="eq:control_law"></a>
\\[ \begin{aligned} 
u_1 &= -(R+B^TQB)^{-1}B^TQAx_1 \tag{7} \\\\ 
&= K_1x_1 
\end{aligned} \\]
<!-- \\[ u_1 = -(R+B^TQB)^{-1}B^TQAx_1 ~~~~(~= K_1x_1~) \tag{7} \\] -->

where $K_1\coloneqq -(R+B^TQB)^{-1}B^TQA$. We can further substitute the expression of $u_1$
into our potential [(6)](#eq:potential_simplified) so that we have (detailed
calculation in the [Appendix](#marginalization-cost-on-x_1))

<a name="eq:cost_update"></a>
\\[ \begin{aligned} 
\phi_1(x_1) &= x_1^T Q x_1 + (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + 
BKx_1) \\\\ 
&= x_1^T(Q+A^TQA - K_1^TB^TQA)x_1 \\\\ 
&= x_1^T V_1 x_2 \tag{8} 
\end{aligned} \\]
where $V_1\coloneqq Q+A^TQA - K_1^TB^TQA$.

As illustrated in [Figure 4](#fig_eliminate_u), through the above steps, we can eliminate variable
$x_2$, $u_2$ as well as three factors marked in red, and replace them with a new factor on $x_1$
marked in blue, with potential $x_1^TV_1x_1$ , which represents the marginalized cost on state
$x_1$.
<!-- ************************ BAYES NET ************************ -->
<a id="sec:elim_bayes"></a>
### Turning into a Bayes Network
By eliminating all the variables from right to left, we can get a Bayes network
as shown in [Figure 5](#fig_bayes_net). Everytime we eliminate an older state
and control, we simply repeat the steps in [Eliminate a state](#eliminate-a-state) and [Eliminate a control](#eliminate-a-control): we express the
older state $x_{k+1}$ with the dynamics model, and express the control $u_k$ as
a function of state $x_k$, then generate a new factor on $x_k$ representing the
"cost-to-go" function $x_k^TV_kx_k$.

Eliminating a general state, $x_{k+1}$, and control $u_k$, we obtain the recurrence relations:
<a name="eq:control_update_k"></a>
\\[ K_k = -(R+B^TV_{k+1}B)^{-1}B^TV_{k+1}A \tag{9} \\]
<a name="eq:cost_update_k"></a>
\\[ V_k = Q+A^TV_{k+1}A - K_k^TB^TV_{k+1}A \tag{10} \\]
with $V_{T}=Q$ is the cost at the last time step.
</div> <!-- scrollablecontent -->
<!-- ************************ END SCROLLABLE ELIMINATION DESCRIPTION ************************ -->

## Connection with Linear Quadratic Regulator
In \cite{}, the control law and cost function for LQR are given by

\\[ u_k = K_kx_k \\]
<a name="eq:control_update_k_ricatti"></a>
\\[ K_k = -(R+B^TP_{k+1}B)^{-1}B^TP_{k+1}A \tag{11} \\]
<a name="eq:cost_update_k_ricatti"></a>
\\[ P_k = Q+A^TP_{k+1}A - K_k^TB^TP_{k+1}A \tag{12} \\]

with $P_k$ commonly referred to as the solution to the dynamic Ricatti equation.

Note that [(11)](#eq:control_update_k_ricatti) and [(12)](#eq:cost_update_k_ricatti) correspond to
the same results as we derived in [(9)](#eq:control_update_k) and [(10)](#eq:cost_update_k).

## Intuition
In our factor graph representation, it is becomes obvious that the dynamic Ricatti equation
solutions, $P_k$, correspond to the total cost at the state $x_k$ that will be accrued for the remainder
of the trajectory and control assuming optimal control after $x_k$.  Specifically, "cost-to-go"
$\phi_k$ is given by 
\\[ \phi_k = x_k^TP_kx_k \\]
This "cost-to-go" is depicted as a heatmap in [Figure 1](#LQR_example).  The optimal control, $K_k$,
represents a balance between achieving a small "cost-to-go" next time step ($B^TP_{k+1}B$) and exerting a small
amount of control this time step ($R$).

## GTSAM Implementation
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
    PRIOR_NOISE = gtsam.noiseModel_Constrained.All(n)
    DYNAMICS_NOISE = gtsam.noiseModel_Constrained.All(n)
    Q_NOISE = gtsam.dynamic_cast_noiseModel_Diagonal_noiseModel_Gaussian(
        gtsam.noiseModel_Gaussian.Information(Q))
    R_NOISE = gtsam.dynamic_cast_noiseModel_Diagonal_noiseModel_Gaussian(
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
    graph.add(X[0], np.eye(n), X0, PRIOR_NOISE)

    # Add dynamics constraint as ternary factor
    #   A.x1 + B.u1 - I.x2 = 0
    for i in range(num_time_steps-1):
        graph.add(X[i], A, U[i], B, X[i+1], -np.eye(n),
                  np.zeros((n)), DYNAMICS_NOISE)

    # Add cost functions as unary factors
    for x in X:
        graph.add(x, np.eye(n), np.zeros(n), Q_NOISE)
    for u in U:
        graph.add(u, np.eye(p), np.zeros(p), R_NOISE)

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
## Appendix
### Eliminate $u_1$
Setting the derivative of [(dumynumber)](#eq:) wrt $u_1$ to zero:
\\[ Ru_1 + B^TQ(Ax_1+Bu_1) = 0 \\]
we have
\\[ \begin{aligned} 
    & (R+B^TQB)u_1 + B^TQAx_1 = 0 \\\\ 
    & x_1 = -(R+B^TQZB)^{-1}B^TQAx_1 
\end{aligned} \\]

### Marginalization Cost on $x_1$
By substituting [(7)](#eq:control_law) into [(6)](#eq:potential_simplified), we have
\\[ \begin{aligned} 
    \phi(x_1) &= x_1^T Q x_1 + (K_1x_1)^T RK_1x_1 + (Ax_1 + BKx_1)^T Q (Ax_1 + BKx_1) \\\\ 
    &= x_1^T(Q+ K_1^TRK_1 + A^TQA + K_1^TB^TQB - K_1^TB^TQA - A^TQBK_1)x_1  \\\\ 
    &= x_1^T[Q + A^TQA + K_1^T(R+B^TQB)K_1 - K_1^TB^TQA - A^TQBK_1]x_1 \\\\ 
    &= x_1^T(Q + A^TQA + A^TQBK_1 - K_1^TB^TQA - A^TQBK_1)x_1 \\\\ 
    &= x_1^T(Q + A^TQA - K_1^TB^TQA)x_1 
\end{aligned} \\]

### Least Squares Implementation in GTSAM
GTSAM uses one of two methods for solving the least squares problems that appear
in eliminating factor graphs: Cholesky Factorization and QR Factorization.  We
will see that, for the LQR formulation, both arrive at the same solution
matching the dynamic Ricatti equation solution.
#### Cholesky Factorization
To solve the linear least square problem $\argmin\limits_x\|\|Ax-b\|\|_2^2$, the solution is $A^TAx=A^Tb$. Cholesky factorization computes a factorization on $A^TA$ such that $A^TA=R^TR$. In GTSAM, Cholesky factorization is performed on each clique. In each clique, GTSAM first performs variable elimination with the strict constrain, then perform Cholesky factorization on rest of the variables. In the clique marked in red in Figure\ref{}, 
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
where $D_1 = $, $K_1=$, and $V_1=$

#### QR Factorization
The process is illustrated in [Figure 6](#fig:elimination) along with the noise matrices and elimination matrices corresponding to the current states of the graphs.

% TODO: explain this...
<a name="fig:elimination"></a>
<figure class="center">
  <img src="/assets/images/lqr_control/ FG figs and clean u"
    alt="Elimination" />
    <figcaption><b>Figure 6</b> Elimination</figcaption>
</figure>

In GTSAM, the variable eliminations can be done using QR or Cholesky factorization.  Block QR Factorization is non-trivial to follow so, for demonstrative purposes, we can show what the result of factorization will be in symbolic block form algebraically using the "completing the square" technique.  Taking, for example, the elimination of $u_0$,

\\[ \begin{aligned} \scriptstyle 
    J(u_0, x_0) & \scriptstyle=~ \|\| P_1^{1/2}Bu_0 ~+~ P_1^{1/2}Ax_0\|\|^2_2 ~+~ \|\|R^{1/2}u_0\|\|^2_2 \qquad+ \|\|Q^{1/2}x_0\|\|^2_2 \\\\ 
        & \scriptstyle=~ u_0^T(B^TP_1B+R)u_0 ~+~ 2u_0^TB^TP_1Ax_0 ~+~ x_0^TA^TP_1Ax_0 \qquad+ x^TQx \\\\ 
        & \scriptstyle=~ u_0^T(B^TP_1B+R)u_0 ~+~ 2u_0^T(B^TP_1B+R)^{1/2}(B^TP_1B+R)^{-1/2}B^TP_1Ax_0 ~+~ x_0^TA^TP_1Ax_0 \qquad+ x_0^TQx_0 \\\\ 
        & \scriptstyle=~ \|\|(B^TP_1B+R)^{1/2}u_0 ~+~ (B^TP_1B+R)^{-1/2}B^TP_1Ax_0\|\|^2_2 ~-~ x_0^T(A^TP_1^TB(B^TP_1B+R)^{-T}B^TP_1A)x_0 ~+~ x_0^TA^TP_1Ax_0 ~+~ x^TQx  \\\\ 
        & \scriptstyle=~ \|\|(B^TP_1B+R)^{1/2}u_0 ~+~ (B^TP_1B+R)^{-1/2}B^TP_1Ax_0\|\|_2^2 ~+~ \|\|(Q - A^TP_1^TB(B^TP_1B+R)^{-T}B^TP_1A ~+~ A^TP_1A)^{1/2}x_0\|\|_2^2  \\\\ 
        & \scriptstyle=~ \|\|(B^TP_1B+R)^{1/2}u_0 ~+~ (B^TP_1B+R)^{-1/2}B^TP_1Ax_0\|\|_2^2 ~+~ \|\|(Q - K_0^TB^TP_1A ~+~ A^TP_1A)^{1/2}x_0\|\|_2^2 \\\\ 
        & \scriptstyle=~ \|\|V\_{0,0}u_0 ~+~ V\_{0,1}x_0\|\|_2^2 ~+~ \|\|P_0^{1/2}x_0\|\|_2^2 
\end{aligned} \\]

Observe also


<!-- \begin{comment}
    Since the factor graph contains both both strict and non-strict constraints, we introduce Lagrangian multipliers $z_0$, $z_1$, $z_2$ to solve the problem. They each correspond to the constraints $x_0=X_0$, $x_1=Ax_0+Bu_0$, $x_2=Ax_1+Bu_1$.\\
    
    We will analyze the problem of 3 time steps ([Figure 2](#fg_scratch)) as an example. The elimination order is $u_2, x_2, z_2, u_1, x_1, z_1, u_0, x_0, z_0$. The corresponding matrix is
    \begin{equation*}
    \begin{bmatrix}
    2R & & & & & & & & \\
    & 2Q & -I & & & & & & \\
    & -I & & B & A & & & & \\
    & & B^T & 2R & & & & & \\
    & & A^T & & 2Q & -I & & & \\
    & & & & -I & & B & A & \\
    & & & & & B^T & 2R & & \\
    & & & & & A^T & & 2Q & -I \\
    & & & & & & & -I & \\
    \end{bmatrix}
    \begin{bmatrix}
    u_2\\x_2\\z_2\\u_1\\x_1\\z_1\\u_0\\x_0\\z_0\\
    \end{bmatrix}
    =
    \begin{bmatrix}
    0\\\vdots\\\vdots\\\vdots\\\vdots\\0\\-X_0\\
    \end{bmatrix}
    \end{equation*}
    
    % TODO: talk with Frank about what is the standard way to solve constraint optimization problem.
    % TODO: it seems solving it with Cholesky factorization will result in complex entries
    
    Applying Gauss-Jordan elimination on the matrix, we have
    \begin{equation*}
    \begin{bmatrix}
    I & & & & & & & & \\
    & I & -(2Q)^{-1} & & & & & & \\
    & & I & -2QB & -2QA & & & & \\
    & & & I & -K_1 & & & & \\
    & & & & I & -\frac{1}{2}V_1^{-1} & & & \\
    & & & & & I & -2V_1B & -2V_1A &  \\
    & & & & & & I & -K_0 &  \\
    & & & & & & & I & -\frac{1}{2}V_0^{-1}\\
    & & & & & & & & I \\
    \end{bmatrix}
    \begin{bmatrix}
    u_2\\x_2\\z_2\\u_1\\x_1\\z_1\\u_0\\x_0\\z_0\\
    \end{bmatrix}
    =
    \begin{bmatrix}
    0\\\vdots\\\vdots\\\vdots\\\vdots\\0\\-2V_0X_0\\
    \end{bmatrix}
    \end{equation*}
\end{comment} -->

### Final Symbolic Expressions of Factor Graph Evaluation
In the above solution, we have
\\[ \begin{aligned} 
K_1 &= -(R+B^TQB)^{-1}B^TQA\\\\ 
P_1 &= Q+A^TQA + A^TQBK_1\\\\ 
K_0 &= -(R+B^TV_1B)^{-1}B^TV_1A\\\\ 
P_0 &= Q + A^T V_1 A + A^T V_1 B K_0
\end{aligned} \\]

% Note that, by switching the elimination order of $x_0$ and $z_0$, solving for $V_0$ is unnecessary.

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
which match the algorithm for solving the finite-horizon discrete-time LQR problem.  As the number
of time steps grows, the solution for $V_0$ approaches the stationary solution to the algebraic
Ricatti equation and the solution for $K_0$ approaches the solution to the infinite-horizon
discrete-time LQR problem.


<script>
    var slideIndex = 1;
    showSlides(slideIndex);

    // Next/previous controls
    function plusSlides(n) {
    showSlides(slideIndex += n);
    }

    // Thumbnail image controls
    function currentSlide(n) {
    showSlides(slideIndex = n);
    }

    function setNodesHidden(nodes, hidden) {
    for (node of nodes) {
        node.hidden = hidden
    }
    }

    function showSlides(n, triggeredByScroll) {
    var i;
    var slides = document.getElementsByClassName("mySlides");
    var dots = document.getElementsByClassName("dot");
    if (n > slides.length) {slideIndex = 1}
    if (n < 1) {slideIndex = slides.length}
    for (i = 0; i < slides.length; i++) {
        slides[i].style.display = "none";
    }
    for (i = 0; i < dots.length; i++) {
        dots[i].className = dots[i].className.replace(" active", "");
    }
    slides[slideIndex-1].style.display = "block";
    dots[slideIndex-1].className += " active";

    var scrollable = document.getElementById("sec:elim_scrollable");
    var scrollLoc_state = document.getElementById("sec:elim_state").offsetTop - scrollable.offsetTop;
    var scrollLoc_ctrl = document.getElementById("sec:elim_ctrl").offsetTop - scrollable.offsetTop;
    var scrollLoc_bayes = document.getElementById("sec:elim_bayes").offsetTop - scrollable.offsetTop;
    var scroll_cur = scrollable.scrollTop;
    var scrollLoc;
    switch(slideIndex) {
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

    document.getElementById("sec:elim_scrollable").addEventListener("scroll", function (event) {
        var scrollable = document.getElementById("sec:elim_scrollable");
        var scrollLoc_state = document.getElementById("sec:elim_state").offsetTop - scrollable.offsetTop;
        var scrollLoc_ctrl = document.getElementById("sec:elim_ctrl").offsetTop - scrollable.offsetTop;
        var scrollLoc_bayes = document.getElementById("sec:elim_bayes").offsetTop - scrollable.offsetTop;
        
        var scroll = this.scrollTop;
        if (scroll < scrollLoc_ctrl) {
            if (slideIndex > 2) {showSlides(slideIndex=1, true)}
        }
        else if (scroll < (scrollable.scrollHeight - scrollable.offsetHeight)) {
            if ((slideIndex < 3) || (slideIndex > 4)) {showSlides(slideIndex=3, true)}
        }
        else {
            if ((slideIndex < 5)) {showSlides(slideIndex=5, true)}
        }
    });
</script>