#!/usr/bin/env python
"""
Create factor graphs for LQR control
Author: Frank Dellaert, Gerry Chen, and Yetong Zhang
"""

import gtsam
import numpy as np

import matplotlib.pyplot as plt
from dynamics_lti import create_lti_fg, plot_trajectory, solve_lti_fg

def add_lqr_costs_fg(graph, X, U, Q, R, x_goal=np.array([])):
    '''Adds LQR quadratic costs to states and controls in a factor graph
    Arguments:
        graph: a factor graph describing system dynamics
        X: a list of keys for the states
        U: a list of keys for the controls
        Q: nxn state cost matrix
        R: pxp control cost matrix
        x_goal: desired goal state (may be n-vector or Txn)
    Returns:
        graph: linear factor graph of the LQR problem
        X: keys for the states
        U: keys for the controls
    '''
    T = len(X)
    n = np.size(Q, 0) # dimension of state space
    p = np.size(R, 0) # dimension of control space

    # condition x_goal
    if x_goal.size == 0:
        x_goal = np.zeros((len(X), n))
    if (x_goal.size == n and np.issubdtype(x_goal[0], np.number)):
        x_goal = np.repeat(np.reshape(x_goal, (1, n)), T, axis=0)
    if x_goal.shape != (len(X), n):
        raise ValueError('Goal position array is not the right shape, must either be n-vector or'+
                         ' (num_time_steps, n)')

    # noises
    q_noise = gtsam.dynamic_cast_noiseModel_Diagonal_noiseModel_Gaussian(
        gtsam.noiseModel_Gaussian.Information(Q))
    r_noise = gtsam.dynamic_cast_noiseModel_Diagonal_noiseModel_Gaussian(
        gtsam.noiseModel_Gaussian.Information(R))
    # note: GTSAM 4.0.2 python wrapper doesn't have 'Information'
    # wrapper, use this instead if you are not on develop branch:
    #   `gtsam.noiseModel_Gaussian.SqrtInformation(np.sqrt(Q)))`

    # set cost functions as unary factors
    for i, x in enumerate(X):
        graph.add(x, np.eye(n), x_goal[i, :], q_noise)
    for u in U:
        graph.add(u, np.eye(p), np.array([0.]), r_noise)

    return graph, X, U

def create_lqr_fg(A, B, Q, R, X0=np.array([0., 0.]), num_time_steps=500,
                  x_goal=np.array([0., 0.])):
    '''Creates a factor graph for solving a discrete, finite horizon LQR problem
    given system dynamics in state space representation.
    Arguments:
        A: nxn state transition matrix
        B: nxp control input matrix
        Q: nxn state cost matrix
        R: pxp control cost matrix
        X0: initial state (n-vector)
        num_time_steps: number of time steps
        x_goal: desired goal state (may be n-vector or Txn)
    Returns:
        graph: linear factor graph of the LQR problem
        X: keys for the states
        U: keys for the controls
    '''
    graph, X, U = create_lti_fg(A, B, X0=X0, num_time_steps=num_time_steps)
    graph, X, U = add_lqr_costs_fg(graph, X, U, Q, R, x_goal=x_goal)
    return graph, X, U

def solve_lqr_fg(graph, X, U):
    '''Solves an LQR problem given in factor graph form.
    Arguments:
        graph: a factor graph
        X: a list of keys for the states
        U: a list of keys for the controls
        toPlot: bool whether or not you want to visualize results
    Returns:
        x_sol: an array of states
        u_sol: an array of controls
    '''
    return solve_lti_fg(graph, X, U)

def solve_lqr(A, B, Q, R, X0=np.array([0., 0.]), num_time_steps=500,
              x_goal=np.array([0., 0.])):
    '''Solves a discrete, finite horizon LQR problem given system dynamics in
    state space representation.
    Arguments:
        A: nxn state transition matrix
        B: nxp control input matrix
        Q: nxn state cost matrix
        R: pxp control cost matrix
        X0: initial state (n-vector)
        num_time_steps: number of time steps
        x_goal: desired goal state (may be n-vector or Txn)
    Returns:
        x_sol: an array of states
        u_sol: an array of controls
    '''
    graph, X, U = create_lqr_fg(A, B, Q, R, X0, num_time_steps, x_goal)
    return solve_lqr_fg(graph, X, U)

def get_return_cost(graph, key):
    '''Returns the value function value at variable `key` given a graph which
        goes up and including `key`, but no further (i.e. all time steps after
        `key` have already been eliminated).  "Return Cost" aka "Cost-to-go" aka
        "Value Function".
    Arguments:
        graph: factor graph in LTI form
        key: key in the factor graph for which we want to obtain the return cost
    Returns:
        return_cost: return cost, an nxn array where `n` is dimension of `key`
    '''
    new_fg = gtsam.GaussianFactorGraph()
    for i in range(graph.size()): # loop through all factors
        f = graph.at(i)
        if (f.keys().size() == 1) and (f.keys().at(0) == key): # unary factor on `key`
            new_fg.push_back(f)
    sol_end = new_fg.eliminateSequential()
    return sol_end.back().information()

def get_k_and_v(graph, X, U):
    '''Finds optimal control law given by $u=Kx$ and value function $Vx^2$ aka
        cost-to-go which corresponds to solutions to the algebraic, finite
        horizon Ricatti Equation.  K is Extracted from the bayes net and V is
        extracted by incrementally eliminating the factor graph.
    Arguments:
        graph: factor graph containing factor graph in LQR form
        X: list of state Keys
        U: list of control Keys
    Returns:
        K: optimal control matrix, shape (T-1, 1)
        V: value function, shape (T, 1)
            TODO(gerry): support n-dimensional state space
    '''
    T = len(X)
    # Find K and V by using bayes net solution
    marginalized_fg = graph
    K = np.zeros((T-1, 1))
    V = np.zeros((T, 1))
    V[-1] = get_return_cost(marginalized_fg, X[-1])
    for i in range(len(U)-2, -1, -1): # traverse backwards in time
        ordering = gtsam.Ordering()
        ordering.push_back(X[i+1])
        ordering.push_back(U[i])

        bayes_net, marginalized_fg = marginalized_fg.eliminatePartialSequential(ordering)
        V[i] = get_return_cost(marginalized_fg, X[i])
        K[i] = bayes_net.back().S()

    return K, V

def main():
    '''Solves open loop LQR problem using factor graph for a spring-mass system
    '''
    # Simulation setup
    del_t = 0.005
    tf = 5
    num_time_steps = int(tf / del_t)
    t = np.arange(num_time_steps)*del_t

    # Problem setup
    K = 1
    m = .5
    X0 = np.array([1, 1], dtype=np.float)
    x_goal = np.array([0, 0], dtype=np.float)
    Q = np.eye(2)
    R = np.eye(1)

    # Matrices
    A = np.array([[1., del_t],
                  [-K/m*del_t, 1.]])
    B = np.array([[0.],
                  [del_t/m]])

    # solve
    x_sol, u_sol = solve_lqr(A, B, Q, R, X0, num_time_steps=num_time_steps, x_goal=x_goal)

    # plot
    plot_trajectory(t, x_sol, u_sol, state_labels=['position', 'velocity'])
    plt.suptitle('LQR control of a spring-mass system by GTSAM')
    plt.show()

if __name__ == '__main__':
    main()
