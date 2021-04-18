#!/usr/bin/env python
"""
Create factor graphs for LTI system dynamics of the form x_{t+1} = Ax_t + Bu_t
Author: Gerry Chen, Yetong Zhang, and Frank Dellaert
"""

import gtsam
import matplotlib.pyplot as plt
import numpy as np

def create_lti_fg(A, B, X0=np.array([]), u=np.array([]), num_time_steps=500):
    '''Creates a factor graph with system dynamics constraints in state-space
            representation:
            x_{t+1} = Ax_t + Bu_t
    Arguments:
        A: nxn State matrix
        B: nxp Input matrix
        X0: initial state (n-vector)
        u: Txp control inputs (optional, if not specified, no control input will
            be used)
        num_time_steps: number of time steps, T, to run the system
    Returns:
        graph, X, U: A factor graph and lists of keys X & U for the states and
            control inputs respectively
    '''
    # Create noise models
    prior_noise = gtsam.noiseModel.Constrained.All(np.size(A, 0))
    dynamics_noise = gtsam.noiseModel.Constrained.All(np.size(A, 0))
    control_noise = gtsam.noiseModel.Constrained.All(1)

    # Create an empty Gaussian factor graph
    graph = gtsam.GaussianFactorGraph()

    # Create the keys corresponding to unknown variables in the factor graph
    X = []
    U = []
    for i in range(num_time_steps):
        X.append(gtsam.symbol('x', i))
        U.append(gtsam.symbol('u', i))

    # set initial state as prior
    if X0.size > 0:
        if X0.size != np.size(A, 0):
            raise ValueError("X0 dim does not match state dim")
        graph.add(X[0], np.eye(X0.size), X0, prior_noise)

    # Add dynamics constraint as ternary factor
    #   A.x1 + B.u1 - I.x2 = 0
    for i in range(num_time_steps-1):
        graph.add(X[i], A, U[i], B, X[i+1], -np.eye(np.size(A, 0)),
                  np.zeros((np.size(A, 0))), dynamics_noise)

    # Add control inputs
    for i in range(len(u)):
        if np.shape(u) != (num_time_steps, np.size(B, 1)):
            raise ValueError("control input is wrong size")
        graph.add(U[i], np.eye(np.size(B, 1)), u[i, :], control_noise)

    return graph, X, U

def solve_lti_fg(graph, X, U):
    '''Solves a linear factor graph describing system dynamics and/or control.
    Arguments:
        graph: a factor graph
        X: a list of keys for the states
        U: a list of keys for the controls
        toPlot: bool whether or not you want to visualize results
    Returns:
        x_sol: an array of states
        u_sol: an array of controls
    '''
    # Solve
    result = graph.optimize()
    # print("\nFinal Result:\n{}".format(result))
    num_time_steps = len(X)
    x_sol = np.zeros((num_time_steps, 2))
    u_sol = np.zeros((num_time_steps, 1))
    for i in range(num_time_steps):
        x_sol[i, :] = result.at(X[i])
        u_sol[i] = result.at(U[i])
    return x_sol, u_sol

def solve_lti(A, B, X0=np.array([0., 0.]), u=np.array([]), num_time_steps=500):
    '''Simulates an LTI system with system dynamics given by:
            x_{t+1} = Ax_t + Bu_t
    Arguments:
        A: nxn State matrix
        B: nxp Input matrix
        X0: initial state (n-vector)
        u: Txp control inputs (optional, if not specified, no control input will
            be used)
        num_time_steps: number of time steps, T, to run the system
    Returns:
        x_sol: an array of states
        u_sol: an array of controls
    '''
    graph, X, U = create_lti_fg(A, B, X0=X0, u=u, num_time_steps=num_time_steps)
    return solve_lti_fg(graph, X, U)

def plot_trajectory(t, x, u, state_labels=None, control_labels=None):
    '''Utiliy function plots state and control trajectories
    Arguments:
        t: times
        x: states
        u: controls
        state_labels: list of labels for the state vector x (optional)
        control_labels: list of labels for the control vector u (optional)
    '''
    plt.subplot(2, 1, 1)
    plt.plot(t, x)
    plt.xlabel('time')
    plt.ylabel('state trajectory')
    if state_labels:
        plt.legend(state_labels)
    plt.subplot(2, 1, 2)
    plt.plot(t, u)
    plt.xlabel('time')
    plt.ylabel('control input')
    if control_labels:
        plt.legend(control_labels)

def main():
    '''runs a simulation of a spring-mass system in 1D using factor graph
    '''
    # Simulation setup
    del_t = 0.01
    tf = 25
    num_time_steps = int(tf / del_t)
    t = np.reshape(np.arange(num_time_steps)*del_t, (num_time_steps, 1))

    # Problem setup
    K = 1
    m = .5
    X0 = np.array([1, 1], dtype=np.float)
    u = np.sin(t)*.5
    u[0:int(7/del_t)] = 0

    # Matrices
    A = np.array([[1., del_t],
                  [-K/m*del_t, 1.]])
    B = np.array([[0.],
                  [del_t/m]])

    # solve
    x_sol, u_sol = solve_lti(A, B, X0, u=u, num_time_steps=num_time_steps)

    # plot
    plot_trajectory(t, x_sol, u_sol, state_labels=['position', 'velocity'])
    plt.suptitle('LQR control of a spring-mass system by GTSAM')
    plt.show()

if __name__ == '__main__':
    main()
