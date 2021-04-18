import numpy as np
import matplotlib.pyplot as plt
import gtsam

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
    prior_noise = gtsam.noiseModel.Constrained.All(n)
    dynamics_noise = gtsam.noiseModel.Constrained.All(n)
    q_noise = gtsam.noiseModel.Gaussian.Information(Q)
    r_noise = gtsam.noiseModel.Gaussian.Information(R)

    # Create an empty Gaussian factor graph
    graph = gtsam.GaussianFactorGraph()

    # Create the keys corresponding to unknown variables in the factor graph
    X = []
    U = []
    for i in range(num_time_steps):
        X.append(gtsam.symbol('x', i))
        U.append(gtsam.symbol('u', i))

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

def main():
    A = np.eye(3)
    B = np.diag([0.15,0.10,0.05])
    Q = np.eye(3)
    R = np.eye(3)
    x_sol, u_sol = solve_lqr(A, B, Q, R, np.ones(3), num_time_steps=100)
    # plot solution
    plt.figure()
    plt.subplot(211)
    plt.plot(x_sol)
    plt.subplot(212)
    plt.plot(u_sol)
    plt.show()

if __name__ == '__main__':
    main()