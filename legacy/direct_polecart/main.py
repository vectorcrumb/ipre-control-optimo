import cvxpy as cp
import numpy as np


# State dimension
n = 4
m = 1
# Time nodes
N = 10
# Final and initial time
tf = 2
ti = 0
# Boundary constraints @ tf
q1_obj = 1
q2_obj = np.pi
# Path constraints
dmax = 2
umax = 20
# Problem parameters
m1 = 1
m2 = 0.3
g = 9.81
l = 0.5
# Define nonlinear dynamics
def dynamics(x, u):
    q1, q2, dq1, dq2 = x
    avcd = np.sin(4)
    q2s = np.sin(q2)
    dx = [
        dq1,
        dq2,
        (l * m2 * np.sin(q2) * (dq2 ** 2) + m2 * g * np.cos(q2) * np.sin(q2) + u) / (m1 + m2 * (1 - (np.cos(q2)) ** 2)),
        -(l*m2*np.cos(q2)*np.sin(q2)*(dq2 ** 2) + u*np.cos(q2) + (m1 + m2)*g*np.sin(q2) ) / (l*m1 + l*m2*(1 - (np.cos(q2)) ** 2)),
    ]
    return dx

# Time nodes
t = np.linspace(ti, tf, N)
# State and control decision variables
x = cp.Variable((N + 1, n))
u = cp.Variable((N, m))
# Declare constraint and cost function
cost = 0
constr = []
# Create boundary constraints
constr += [x[N,:] == np.array([q1_obj, q2_obj, 0, 0])]
constr += [x[0,:] == np.array([0, 0, 0, 0])]
# Create path and dynamic constraints
for k in range(0, N - 1):
    # Compute current timestep from time vector
    hk = t[k + 1] - t[k]
    # Add quadrature integral cost for current step
    cost += (hk / 2) * (u[k]**2 + u[k + 1]**2)
    # Add system dynamics constraint for current step
    # constr += [x[k+1,:] == dynamics(x[k,:], u[k,:])]
    constr += [(hk / 2) * (dynamics(x[k+1,:], u[k+1]) + dynamics(x[k,:], u[k])) == x[k+1,:] - x[k,:]]
    # Add path constraints
    constr += [
        -dmax <= x[k, 0],
        x[k,0] <= dmax,
        -umax <= u[k],
        u[k] <= umax
    ]

# Solve problem
problem = cp.Problem(cp.Minimize(cost), constraints=constr)
prob.solve(solver=cp.MOSEK, verbose=True)







# # Create two scalar optimization variables.
# x = cp.Variable()
# y = cp.Variable()

# # Create two constraints.
# constraints = [x + y == 1,
#                x - y >= 1]

# # Form objective.
# obj = cp.Minimize((x - y)**2)

# # Form and solve problem.
# prob = cp.Problem(obj, constraints)
# prob.solve(solver=cp.MOSEK, verbose=True)  # Returns the optimal value.
# print("status:", prob.status)
# print("optimal value", prob.value)
# print("optimal var", x.value, y.value)