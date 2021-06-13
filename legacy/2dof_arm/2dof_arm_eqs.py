#------------------------------------------------------------------------------#
# 2-DOF Robotic Arm - Forward Dynamics Equations
# execute typing: python 2dof_arm_eqs.py

#import numpy as np
from numpy import *


# --- Geometric and Inertial Parameters --- 
# Links length
l1=0.5
l2=0.5

# Links mass and inertias
m1=1
m2=1
m3=1
I3=m3*0.1**2

# Gravity acceleration
g = 9.81

# Joint friction  coefficients
b1 = 1e-3
b2 = 1e-3

# Joint variables (position q, velocity qd, torque tau)
q1 = pi/6
q2 = pi/6
qd1 = pi/60
qd2 = pi/60
tau1 = 1
tau2 = 2
   
l1_2 = l1*l1
l2_2 = l2*l2
l1l2 = l1*l2
cq1 = cos(q1)
cq2 = cos(q2)
sq2 = sin(q2)
cq12 = cos(q1+q2)

m11 = (1/3)*(m1+3*(m2+m3))*l1_2+(m2+2*m3)*l1l2*cq2+(1/3)*(m2+3*m3)*l2_2+I3
m12 = (1/2)*(m2+2*m3)*l1l2*cq2+(1/3)*(m2+3*m3)*l2_2+I3
m21 = m12
m22 = (1/3)*(m2+3*m3)*l2_2+I3

M = array([[m11,m12],
           [m21,m22]])

Minv = linalg.inv(M)

#print(M.dot(Minv))

ca = (m2+2*m3)*l1l2*sq2
c11 = -ca*qd2
c12 = -(ca/2)*qd2
c21 =  (ca/2)*qd1
c22 = 0

C = array([[c11,c12],
           [c21,c22]])
qd = array([[qd1],
            [qd2]])
Cq = C.dot(qd)

g1 = (1/2)*((m1+2*m2+2*m3)*l1*cq1+(m2+2*m3)*l2*cq12)*g
g2 = (1/2)*(m2+2*m3)*l2*cq12*g

G = array([[g1],
           [g2]])

B = array([[b1*qd1],
           [b2*qd2]])

tau = array([[tau1],
             [tau2]])
             
# Joint acceleration
qdd = Minv.dot(tau-Cq-G-B)

print(qdd)


             
# def main():
                
# if __name__ == '__main__': main()