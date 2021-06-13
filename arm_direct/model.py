import numpy as np

## Model parameters
# Gravity
g = 9.81
# Link length
l1 = 0.5
l2 = 0.5
# Link mass and inertia
m1 = 1
m2 = 1
m3 = 1
I3 = m3 * (0.1 ** 2)
# Friction coefficients
b1 = 1
b2 = 1
# State initial conditions
q1 = 0
q2 = 0
dq1 = 0
dq2 = 0
# Input initial conditions
tau1 = 0
tau2 = 0