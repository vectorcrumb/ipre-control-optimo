from scipy.optimize import minimize
from scipy.optimize.optimize import OptimizeResult
from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt


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
# Time nodes
t = np.linspace(ti, tf, N)

# Define nonlinear dynamics. The unpackaging of variables is needed to work only with numbers, not arrays
def dynamics(x, control):
    q1, q2, dq1, dq2 = x
    u = control[0]
    dx = np.array([
        dq1,
        dq2,
        (l * m2 * np.sin(q2) * (dq2 ** 2) + m2 * g * np.cos(q2) * np.sin(q2) + u) / (m1 + m2 * (1 - (np.cos(q2)) ** 2)),
        -(l*m2*np.cos(q2)*np.sin(q2)*(dq2 ** 2) + u*np.cos(q2) + (m1 + m2)*g*np.sin(q2) ) / (l*m1 + l*m2*(1 - (np.cos(q2)) ** 2)),
    ])
    return dx

# Reconstruct state from decision vector
def get_state(xvec, state=None, idx=None, get='x', N=N, n=n, m=m):
    """Get specific state at specific discrete time idx
    Args:
        xvec: Complete decision variable vector (x and u). This should take 
        the form of [x1, x2, ..., xn, u1, u2, ..., um]^T, where xi and ui  
        are vectors of length N, such that xi = [xi_1, xi_2, ..., xi_N]^T
        state: Which state to return. Can be larger than n to obtain a control,
        however it must be smaller than n+m
        idx: Which time index to return. Must be less than N
        get: 'x' returns the states @ idx, 'u' get the control and 'all' returns both
        N: number of time nodes
        n: number of states
        m: number of controls
    """
    assert xvec.shape == (N * (n + m), )
    xmat = xvec.reshape((N, n + m), order='F')
    if state is None and idx is None:
        return xmat
    elif state is None and idx is not None:
        if get == 'x': return xmat[idx, :n]
        elif get == 'u': return xmat[idx, n:]
        elif get == 'all': return xmat[idx, :]
        else: return None
    elif state is not None and idx is None:
        return xmat[:,state]
    elif state is not None and idx is not None:
        return xmat[idx, state]
    else:
        return None
# Lambdas for quick shortcuts to important values
get_uk = lambda x, k: get_state(x, idx=k, get='u')
get_uk1 = lambda x, k: get_state(x, idx=k+1, get='u')
get_xk = lambda x, k: get_state(x, idx=k, get='x')
get_xk1 = lambda x, k: get_state(x, idx=k+1, get='x')

# Declare constraint and cost function
cost = 0
constr = []
bounds = []
# Value constraints are sorted by x1,...,xn,u1,...um. 
# Value constraints are generated later on from this.
boundary_vals = [
    (-dmax, dmax),
    (None, None),
    (None, None),
    (None, None),
    (-umax, umax)
]
# Boundary constraints
constr += [
    {
        # Final state
        'type': "eq",
        'fun': lambda x: get_state(x, idx=N-1) - np.array([q1_obj, q2_obj, 0, 0]),     
    }, {
        # Initial state
        'type': "eq",
        'fun': lambda x: get_state(x, idx=0) - np.array([0, 0, 0, 0]),
    },
]
# Dynamic constrain generator is opened up for ease of debugging
def dynamics_constraint(x, dt, t):
    xk1 = get_xk1(x, t)
    xk = get_xk(x, t)
    uk = get_uk(x, t)
    uk1 = get_uk1(x, t)
    fk1 = dynamics(xk1, uk1)
    fk = dynamics(xk, uk)
    res = (dt/2) * (fk1 + fk) + xk - xk1
    return res
# Dynamics constraints
for k in range(0, N-1):
    hk = t[k+1] - t[k]
    constr += [
        {
            'type': "eq",
            'fun': dynamics_constraint,
            'args': (hk, k,)
        }
    ]
# Objective function
def obj_fun(x):
    obj = 0
    for k in range(0, N-1):
        hk = t[k+1] - t[k]
        obj += (hk / 2) * (get_uk(x, k)**2 + get_uk1(x, k)**2)
    return obj
# Set decision variable boundaries
for i in range(m + n):
    for _ in range(N):
        bounds += [boundary_vals[i]]
# Last bound is u(N-1), which is not a decision variable. Unbound.
bounds[-1] = (None, None)
# Create an initial guess. Quality of solution depends on this guess
x0 = np.zeros((m + n) * N)
x0[:N] = q1_obj * t / (tf - ti)
x0[N:2*N] = q2_obj * t / (tf - ti)
# Solve the problem
res: OptimizeResult = minimize(obj_fun, x0, method='SLSQP', constraints=constr, bounds=bounds, options={'disp': True})
# Test for a solution
if not res.success:
    print("No solution found!")
    exit(0)
# Display solver output
print(res)
opt_sol = get_state(res.x)
# Create interpolators for control and state
state_interpolator = interpolate.interp1d(x=t, y=opt_sol[:,:n], kind='quadratic', axis=0)
control_interpolator = interpolate.interp1d(x=t, y=opt_sol[:,n:], kind='linear', axis=0)
# Plot solution. Interpolation isn't implemented
t_interp = np.linspace(ti, tf, N*10)
plt.plot(t_interp, state_interpolator(t_interp))
plt.plot(t_interp, control_interpolator(t_interp))
plt.legend(["$q_1=x$", "$q_2=\\theta$", "$q_3=\dot{x}$", "$q_4=\dot{\\theta}$", "$u$"], loc='best')
plt.title("State and control vs time")

fig, ax = plt.subplots(3, 1, sharex=True)
fig.suptitle("Optimal Trajectory")
ax[0].set_ylabel("Position [m]")
# ax[0].set_ylim([0, 1.5])
ax[0].plot(t_interp, state_interpolator(t_interp)[:,0])
ax[0].scatter(t, opt_sol[:,0])
ax[1].set_ylabel("Angle [rad]")
# ax[1].set_ylim([-2, 4])
ax[1].plot(t_interp, state_interpolator(t_interp)[:,1])
ax[1].scatter(t, opt_sol[:,1])
ax[2].set_ylabel("Force [Nm]")
# ax[2].set_ylim([-20, 10])
ax[2].plot(t_interp, control_interpolator(t_interp))
ax[2].scatter(t, opt_sol[:,-1])
ax[2].set_xlabel("Time $t_k$")
ax[2].set_xlim([0, 2])

plt.show()