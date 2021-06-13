from scipy.optimize import minimize
from scipy.optimize.optimize import OptimizeResult
from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

n = 4
m = 1

N = 10
ti = 0
tf = 2
t = np.linspace(ti, tf, N)

q_obj = np.array([1, np.pi, 0, 0])
q_init = np.array([0, 0, 0, 0])

boundary_vals = [
    (-2, 2),
    (None, None),
    (None, None),
    (None, None),
    (-20, 20)
]

m1 = 1
m2 = 0.3
g = 9.81
l = 0.5

# Problem dynamics
def dynamics(x, u_):
    q1, q2, dq1, dq2 = x
    u = u_[0]
    dx = np.array([
        dq1,
        dq2,
        (l * m2 * np.sin(q2) * (dq2 ** 2) + m2 * g * np.cos(q2) * np.sin(q2) + u) / (m1 + m2 * (1 - (np.cos(q2)) ** 2)),
        -(l*m2*np.cos(q2)*np.sin(q2)*(dq2 ** 2) + u*np.cos(q2) + (m1 + m2)*g*np.sin(q2) ) / (l*m1 + l*m2*(1 - (np.cos(q2)) ** 2)),
    ])
    return dx

# Objective function
def obj_fun(x):
    x_ = x.reshape((N, n + m), order='F')
    obj = 0
    for k in range(0, N-1):
        hk = t[k+1] - t[k]
        obj += (hk/2) * (np.dot(x_[k, n:], x_[k, n:]) + np.dot(x_[k+1, n:],x_[k+1, n:]))
    return obj

# Generate linear dynamic constraints in between sample points
def dynamics_constraints(x, dt, t):
    x_ = x.reshape((N, n + m), order='F')
    return (dt/2) * (dynamics(x_[t+1, :n], x_[t+1, n:]) + dynamics(x_[t, :n], x_[t, n:])) + x_[t, :n] - x_[t+1, :n]


cost = 0
constr = []
bounds = []

# Boundary constraints (applied for terminal points in trajectory). Expressions are equal to 0
constr += [
    {
        'type': 'eq',
        'fun': lambda x: x.reshape((N, n + m), order='F')[N-1, :n] - q_obj,
    }, 
    {
        'type': 'eq',
        'fun': lambda x: x.reshape((N, n + m), order='F')[0, :n] - q_init,
    },
]

# State boundary constraints. Limit the values for each state and control over the trajectory
for i in range(m + n):
    for _ in range(N):
        bounds += [boundary_vals[i]]

# Dynamics constraints
for k in range(0, N-1):
    hk = t[k+1] - t[k]
    constr += [
        {
            'type': 'eq',
            'fun': dynamics_constraints,
            'args': (hk, k,)
        }
    ]

# There are N samples in the trajectory, each consisting first of the state and then the controls
x0 : np.array = np.zeros((N, n + m))
# Set initial state guess based on a linear interpolation towards the desired position states
x0[:,:n] = np.outer(t, q_obj) / (tf - ti)

res: OptimizeResult = minimize(obj_fun, x0.reshape(N * (n + m), order='F'), method="SLSQP", constraints=constr, bounds=bounds, options={'disp': True})
if not res.success:
    print("No solution found!")
    exit(0)
print(res.x.shape)

sol = res.x.reshape((N, n + m), order='F')

x_interpol = interpolate.interp1d(x=t, y=sol[:, :n], kind='quadratic', axis=0)
u_interpol = interpolate.interp1d(x=t, y=sol[:, n:], kind='linear', axis=0)
t_interpol = np.linspace(ti, tf, 10 * N)

fig, ax = plt.subplots(2, 1, sharex=True)
fig.suptitle("Opt Trajectory")

ax[0].set_ylabel("Position [m]")
ax[0].plot(t_interpol, x_interpol(t_interpol)[:, 0])
ax[0].scatter(t, sol[:, 0])

ax[1].set_ylabel("Angle [rad]")
ax[1].plot(t_interpol, x_interpol(t_interpol)[:, 1])
ax[1].scatter(t, sol[:, 1])



np.save("v2_x.npy", sol[:, :n])
np.save("v2_u.npy", sol[:, n:])



plt.show()