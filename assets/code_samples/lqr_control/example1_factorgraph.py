#!/usr/bin/env python
"""
Solves LQR problem with control law and cost-to-go function using FG and Ricatti and shows
    visually that the solutions are equivalent.
Author: Gerry Chen, Yetong Zhang, and Frank Dellaert
"""
import matplotlib.pyplot as plt
import numpy as np

import lqr
import example0_ricatti as ricatti

def main():
    """Solves optimal control problem for a slightly unstable 1D system using factor graphs.
    Solves for optimal control law and cost-to-go, and plots them.  Also plots Ricatti solution for
        comparison.
    """
    # Problem definition
    A = np.array([[1.03]]) # slightly unstable system :)
    B = np.array([[0.03]])
    Q = np.array([[0.21]])
    R = np.array([[0.05]])
    X0 = np.array([]) # to get optimal control _law_, don't supply initial state
    T = 100

    # Factor graph LQR - these two lines are all you need to solve an LQR problem!
    graph, X, U = lqr.create_fg(A, B, Q, R, X0, num_time_steps=T, x_goal=np.array([0.]))
    K, P = lqr.get_k_and_p(graph, X, U)
    K2 = lqr.get_k(graph, X, U)
    assert np.all(np.abs(K - K2) < 1e-10)

    # ricatti version, for comparison
    K_ricatti, P_ricatti = ricatti.get_k_and_p(T, A, B, Q, R)

    # plotting
    plt.figure(figsize=(18, 5))
    plt.subplots_adjust(top=0.8, wspace=0.3)
    plt.subplot(1, 3, 1)
    plt.plot(range(T), P, 'r-')
    plt.plot(range(T), P_ricatti, 'k:', linewidth=4)
    plt.plot(range(T-1), K, 'g-')
    plt.plot(range(T-1), K_ricatti, 'k:', linewidth=4)
    plt.xlabel('Time step')
    plt.ylabel('$P$ (cost-to-go = $Px^2$)\nOptimal control gain $K$')
    plt.legend(['FG cost-to-go $P$', 'Ricatti cost-to-go $P$',
                'FG control gain $K$', 'Ricatti control gain $K$'])
    plt.title('Cost-to-go and Control Gain\n')
    # value function plots
    ricatti.visualize_cost(A, B, K, P, fig=plt.subplot(1, 3, 2),
                           title='Cost-to-go (Factor Graph)\n')
    ricatti.visualize_cost(A, B, K_ricatti, P_ricatti, fig=plt.subplot(1, 3, 3),
                           title='Cost-to-go (Ricatti Equation)\n')

    plt.suptitle('LQR Control Computed by by Factor Graph (FG) and Ricatti Equation')

    # check to make sure matches ricatti
    if not np.allclose(K, K_ricatti):
        print("Ricatti and FG solutions don't match")
    if not np.allclose(P, P_ricatti):
        print("Ricatti and FG value functions don't match")

    plt.savefig('./LQR_FGvsRicatti', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == '__main__':
    main()
