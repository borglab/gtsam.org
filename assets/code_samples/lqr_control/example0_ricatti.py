#!/usr/bin/env python
"""
Solves and visualizes LQR solution using traditional Ricatti Equation as a "ground truth"
    comparison.
Author: Gerry Chen, Yetong Zhang, and Frank Dellaert
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

def ricatti_update(P, A, B, Q, R, N=None):
    '''Calculates next cost-to-go matrix, P, according to the Discrete Algebraic Ricatti Equation.
    See
    https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Finite-horizon,_discrete-time_LQR
    Arguments:
        P: Previous cost-to-go matrix
        A: state transition matrix
        B: input matrix
        Q: state cost matrix
        R: control cost matrix
        N: cross-term cost matrix
    Returns:
        Pnew: new cost-to-go matrix
        K: control law for new timestep
    '''
    if not N:
        N = np.zeros((np.size(R, 0), np.size(Q, 0)))
    AT = A.transpose()
    BT = B.transpose()
    NT = N.transpose()
    if (R + BT*P*B).ndim == 1:
        inverse_term = 1/(R + BT*P*B)
    else:
        inverse_term = np.linalg.inv(R + BT*P*B)
    K = (AT*P*B + N)*inverse_term
    Pnew = (AT*P*A -
            K*(BT*P*A + NT) +
            Q)
    return Pnew, K

def get_k_and_p_ricatti(T, A, B, Q, R, N=None):
    '''Calculates the control gain, K, and cost-to-go, V, for each time step of a finite horizon LQR
    problem.
    Arguments:
        T: number of time steps
        A: state transition matrix
        B: input matrix
        Q: state cost matrix
        R: control cost matrix
        N: cross-term cost matrix
    Returns:
        K: optimal control law for each time step
        P: cost-to-go for each time step (aka V)
    '''
    P = np.zeros((T, 1))
    K = np.zeros((T-1, 1))
    P[-1] = Q
    for t in range(T-1, 0, -1):
        P[t-1], K[t-1] = ricatti_update(P[t], A, B, Q, R, N)
    return K, P

def visualize_cost(A, B, K, P, fig=None, plottype='heatmap', title='Return Cost'):
    '''Plots the cost-to-go and control law.
    Arguments:
        A: state transition matrix (nxn)
        B: input control matrix (nxp)
        K: optimal control law at each time step (Txn)
        P: cost-to-go at each time step (Tx1)
        fig: optional matplotlib figure/subfigure in which to plot
        plottype: either 'heatmap' or 'level sets'.  If 'heatmap', plots a heatmap of cost-to-go
            with vectors indicating optimal control direction and an example trajectory.  If 'level
            sets', plots level sets of the cost-to-go with state on x-axis, cost on y-axis, and time
            step being parameterized.
        title: optional title of plot
    '''
    T = np.size(K, 0)
    x_vals = np.linspace(-10, 10, 100)
    t_vals = range(T)
    cost_vals = x_vals.transpose()*P*x_vals

    if not fig:
        plt.figure(figsize=(6, 5))

    if plottype == 'heatmap':
        # heat map cost function
        plt.imshow(cost_vals, origin='lower',
                   extent=(x_vals[0], x_vals[-1], t_vals[0], t_vals[-1]),
                   aspect='auto')
        plt.xlabel('x')
        plt.ylabel('time step')
        plt.colorbar().set_label('Cost-to-go')

        # plot quivers
        quiver_inds = slice(0, len(K), 5)
        x_vals_quiver = np.linspace(x_vals[0], x_vals[-1], 15)
        t_vals_quiver = t_vals[quiver_inds]
        u_vals_quiver = -K[quiver_inds]*x_vals_quiver
        q = plt.quiver(x_vals_quiver, t_vals_quiver,
                       B*u_vals_quiver, np.ones(np.shape(u_vals_quiver)), angles='xy')
        plt.quiverkey(q, X=0.07, Y=1.03, U=2,
                      label='Optimal Control Direction', labelpos='E')

        # plot example trajectories
        trajectories = np.zeros((len(t_vals), 3))
        trajectories[0, :] = [x_vals[0], -3, 8]
        for t in range(0, t_vals[-2]):
            u_tmp = - K[t] * trajectories[t, :]
            trajectories[t+1, :] = A*trajectories[t, :] + B*u_tmp
        plt.plot(trajectories, t_vals, 'r-')
        plt.legend(('Example trajectories',), loc="upper right")
    elif plottype == 'level sets':
        # cost function level sets
        cm = plt.cm.copper
        plt.gca().set_prop_cycle(plt.cycler('color',
                                            cm(np.linspace(0, 1, np.size(cost_vals, 0)))))
        plt.plot(x_vals, cost_vals.transpose())
        plt.xlabel('x')
        plt.ylabel('Return cost')
        num_lines = 8
        custom_lines = [Line2D([0], [0], color=cm(i/num_lines), lw=4)
                        for i in range(num_lines)]
        plt.legend(custom_lines, np.linspace(t_vals[0], t_vals[-1], num_lines, dtype=int),
                   title="time step")
    else:
        raise ValueError("plottype must be either 'heatmap' or 'level sets'")

    if title:
        plt.title(title)

def main():
    '''Creates and solves a simple 1D system where the control is particle velocity and state is
    particle position.  Solves LQR problem using Ricatti Equation and plots cost-to-go.
    '''
    A = np.array([1.1])
    B = np.array([0.1])
    Q = np.array([2.])
    R = np.array([1.])
    N = np.array([0.])

    T = 50
    K, P = get_k_and_p_ricatti(T, A, B, Q, R, N)

    plt.figure(figsize=(12, 5))
    visualize_cost(A, B, K, P, fig=plt.subplot(1, 2, 1), title=None, plottype='heatmap')
    visualize_cost(A, B, K, P, fig=plt.subplot(1, 2, 2), title=None, plottype='level sets')
    plt.suptitle('Return cost assuming LQR control for a massless particle '+
                 'moving through molasses (1-D)')
    plt.show()

if __name__ == '__main__':
    main()
