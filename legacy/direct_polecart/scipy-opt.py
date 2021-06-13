from scipy.optimize import minimize, rosen, rosen_der


def obj_fun(x):
    x1, x2 = x
    return (x1 - 1)**2 + (x2 - 2.5)**2

# Ineq constraints are of the form g(x) >= 0
cons = ({
    'type': "ineq",
    'fun': lambda x: x[0] - 2 * x[1] + 2
}, {
    'type': "ineq",
    'fun': lambda x: -x[0] - 2 * x[1] + 6
}, {
    'type': "ineq",
    'fun': lambda x: -x[0] + 2 * x[1] + 2
})

bnds = (
    (0, None),
    (0, None)
)

opts = {
    'disp': True
}

x0 = (2, 0)

res = minimize(obj_fun, x0, method='SLSQP', constraints=cons, bounds=bnds, options=opts)

print(res)