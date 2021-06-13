#------------------------------------------------------------------------------#
# 2-DOF Robotic Arm
# execute typing: python 2dof_arm.py
# http://stackoverflow.com/questions/16044229/how-to-get-keyboard-input-in-pygame
# http://www.pygame.org/docs/ref/key.html
# http://www.tutorialspoint.com/python/python_numbers.htm
# https://docs.python.org/3/library/functions.html#int

# Program flow
# - Initialize state
# - Read user inputs
# - Step model
# - Draw stuff

import os
# try:
#     os.environ["DISPLAY"]
# except:
#     os.environ["SDL_VIDEODRIVER"] = "dummy"

import sys  # find ouf constants in sys.float_info
            # https://docs.python.org/2/library/sys.html#sys.float%5Finfo

import pygame
from pygame.locals import * # Must include this line to get the OpenGL
                            # definition otherwise importing just PyGame 
                            # is fine.

import numpy as np
# from numpy import *

import time

from scipy.integrate import odeint

import matplotlib.pyplot as plt


# --- Physics Variables --- 
# Controls
mv_user = np.array([0.0,0.0]) # This is the user set value for mv  (valve aperture in percent)
mv_out  = np.array([0.0,0.0]) # This is the physical value of the mv (valve aperture in area,  flow rate, or physical signal, e.g. driver current/voltage)
MV_USER_TO_OUT = 2.0 # Convert from 0-1 [%] to 0-2.0 [Nm]
MV_USER_TO_REF = 1.0 # Convert from 0-1 [%] to 0-1.0 [°]
MV_OUT_MIN =  0.0 # [Nm]
MV_OUT_MAX =  2.0 # [Nm]


# State
x = np.array([0.,0.,0.,0.]) # q1, q2, qd1, qd2

TL_LO = 0.0 # State low
TL_HI = 1.0 # State high

# Reference
ref=0

# Model parameters

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
b1 = 1 #1e-3
b2 = 1 #1e-3

# Joint variables (position q, velocity qd, torque tau)
q1  = -0.0*np.pi/2.0
q2  = 0.0
qd1 = 0.0
qd2 = 0.0
tau1 = 0.0
tau2 = 0.0
x[0] = q1
x[1] = q2
x[2] = qd1
x[3] = qd2

# Control mode
auto = 0

# Controller parameters and variables
Kp = 10.0
Ki = 5.0
Kd = 0
error      = 0
error_old  = 0
error_old2 = 0

# --- Simulation Variables ---

# Time counter for sampling time measurement
ti = 0

# Wall-clock time
tw0 = 0

# Simulation sampling time
Ts = 0.001      # This is just a reference value.  The current
                # simulation employs the real elapsed time,
                # i.e. variable sampling period Ts.

# Time scaling
time_scaling = 1.0

# Logger variables
k_data = 0              # Index to entry position of the data storage array
k_data_samples = 10000  # Number of data points to store. The time lenght of the buffer is k_data_samples*Ts ~= 10000*0.02 = 200 s.
record_size = 8        # Size of each record in the data array: time+state vector+referece+manipulated variable = 1+4+1+2
data = np.zeros((k_data_samples,record_size)) # Store in each row: time, state vector (row-wise), reference, manipulated variable 
#http://scipy.github.io/old-wiki/pages/NumPy_for_Matlab_Users.html


# --- Graphics Variables ---
XMAX = 640
YMAX = 480
screen = None

# --- Rotating Arm Variables ---

def Rot2D(theta,tuple_value):
    R=np.array([[np.cos(theta),-np.sin(theta)],
                [np.sin(theta), np.cos(theta)]])
    v = tuple(R.dot(tuple_value))             
    #print(R)
    #print(v)
    return v

def TransformPoints(theta,pos,scale,list_of_tuples):
    #print(list_of_tuples)
    out_list = []
    for tuple_value in list_of_tuples:
        #print(tuple_value)
        v = Rot2D(theta,tuple_value)
        #v = (v[0]+pos[0],v[1]+pos[1])
        out_list += [(v[0]*scale+pos[0],v[1]*scale+pos[1])]
    #print(out_list)
    return out_list

def World2Screen(list_of_tuples):
    # Screen coordinates
    # 0-> Xs
    # |
    # Ys
    # 
    # World coordinates
    # Yw
    # |
    # 0->Xw
    #
    # World-to-Screen Transformation
    # Xs = Xw+XMAX/2
    # Ys =-Yw+YMAX/2

    #print(list_of_tuples)
    out_list = []
    for tuple_value in list_of_tuples:
        #print(tuple_value)
        v = (tuple_value[0]+XMAX/2.0,-tuple_value[1]+YMAX/2.0)
        out_list += [v]
    #print(out_list)
    return out_list
    
def DrawPolygon(surface, vertices, color):
    """
    Draw a wireframe polygon given the screen vertices with the specified color.
    """
    if not vertices:
        return

    if len(vertices) == 2:
        pygame.draw.aaline(surface, color, vertices[0], vertices)
    else:
        pygame.draw.polygon(surface, color, vertices, 0)

def store_data():
    # Store in row 'k' of the 'data' array of size (rows, cols)= (k_data_samples, size(t)+size(x)+size(ref)+size(mv))
    # the following values arranged column wise:
    # Time t, state x, referece ref and manipulated variables mv
    # Data is stored in a circular buffer, aka ring buffer or cyclic buffer
    global data, k_data_samples, k_data
    global tw0, x, ref, mv_out
    
    t = time.time()-tw0
    # x
    #ref = level_ref
    mv  = mv_out
    
    data[k_data,:]=np.r_[t, np.r_[x, np.r_[ref,mv]]]
    k_data = k_data+1
    if k_data == k_data_samples:
        k_data = 0
        
def arrange_data():
    # Arrange the order of the data in the circular buffer from oldest to newest sample
    global data, k_data, k_data_samples
    
    if k_data>0:
        aux1 = data[:k_data,:]                      # Pick the first k_data values
        aux2 = data[-(k_data_samples-k_data):,:]    # Pick the remaining values in the data array
        data = np.r_[aux2,aux1]                        # Vertically stack the rows of aux2 and aux1
    # Check http://scipy.github.io/old-wiki/pages/NumPy_for_Matlab_Users.html
    # for info on array handling.
    
def update_display():
    global x, tau1, tau2
  
    state_str = 'q1: '+str(x[0]*180.0/np.pi)+', q2:'+str(x[1]*180.0/np.pi)+\
                'tau1: '+str(tau1)+', tau2:'+str(tau2)
    
    pygame.draw.rect(screen, (0,0,0) , screen.get_rect(), 0)
    #level_rnd = int(round(x[0],0))
    #print screen.get_rect()
    
    # Draw objects in world coordinates
    BasicPoly = [(0,-1),(10,0),(0,1)] # Robot arm
    TransPoly = TransformPoints(x[0],(0,0),10,BasicPoly)
    DrawPolygon(screen, World2Screen(TransPoly), (0,255,255))
    TransPoly2 = TransformPoints(x[0]+x[1],(TransPoly[1][0],TransPoly[1][1]),5,BasicPoly)
    DrawPolygon(screen, World2Screen(TransPoly2), (255,255,0))
       
    font = pygame.font.SysFont('Arial', 25)
    screen.blit(font.render(state_str, True, (255,0,0)), (100, 100))
    pygame.display.flip()

# Manipulated variable
def u_fun(): #(ref,x):
    global ref, x, mv_user, mv_out
    global error_old, error_old2, Ts
    if auto == 0:
        mv_out = mv_user*MV_USER_TO_OUT
    if auto == 1:
        ref = mv_user*MV_USER_TO_REF
        error = ref-x[0]
        mv_out = mv_out + Kp*(error-error_old) + Ki*Ts*error + Kd*(error-2*error_old+error_old2)/Ts
        if mv_out > MV_OUT_MAX:
            mv_out = MV_OUT_MAX
        elif mv_out < MV_OUT_MIN:
            mv_out = MV_OUT_MIN
        error_old2 = error_old
        error_old = error
    return mv_out

def check_state_bounds():
    global x            # State variable
    global TL_LO, TL_HI # State bounds
    
    for k in range(len(x)):
        if x[k] > TL_HI:   # Check state bounds
            x[k] = TL_HI
        if abs(x[k]-TL_LO) < sys.float_info.epsilon*1e10:
            x[k] = TL_LO

        
# State model
# xd = f(x,u)
def xd_fun(x, t):
    global l1, l2, m1, m2, m3, I3
    global mv_out
    global mv_user # added temporarily - remove when mv_out is computed correctly.
    global tau1, tau2 # added temporarily - remove when mv_out is computed correctly.
    # u = u_fun(ref,x) 
    u = mv_out
        
    q1 = x[0]  # q_1
    q2 = x[1]  # q_2
    qd1 = x[2] # q_dot_1
    qd2 = x[3] # q_dot_2
    
    l1_2 = l1*l1
    l2_2 = l2*l2
    l1l2 = l1*l2
    cq1 = np.cos(q1)
    cq2 = np.cos(q2)
    sq2 = np.sin(q2)
    cq12 = np.cos(q1+q2)

    m11 = (1/3)*(m1+3*(m2+m3))*l1_2+(m2+2*m3)*l1l2*cq2+(1/3)*(m2+3*m3)*l2_2+I3
    m12 = (1/2)*(m2+2*m3)*l1l2*cq2+(1/3)*(m2+3*m3)*l2_2+I3
    m21 = m12
    m22 = (1/3)*(m2+3*m3)*l2_2+I3
   
    M = np.array([[m11,m12],
           [m21,m22]])
    
    Minv = np.linalg.inv(M)

    #print(M.dot(Minv))

    ca = (m2+2*m3)*l1l2*sq2
    c11 = -ca*qd2
    c12 = -(ca/2)*qd2
    c21 =  (ca/2)*qd1
    c22 = 0

    C = np.array([[c11,c12],
               [c21,c22]])
    qd = np.array([[qd1],
                [qd2]])
    Cq = C.dot(qd)

    g1 = (1/2)*((m1+2*m2+2*m3)*l1*cq1+(m2+2*m3)*l2*cq12)*g
    g2 = (1/2)*(m2+2*m3)*l2*cq12*g

    G = np.array([[g1],
               [g2]])

    B = np.array([[b1*qd1],
               [b2*qd2]])

    tau1 = mv_user[0]*20.0
    tau2 = mv_user[1]*20.0
               
    tau = np.array([[tau1],
                 [tau2]])
    
    # Joint acceleration
    qdd = Minv.dot(tau-Cq-G-B)
    #print(G)
    #qdd[0] = 0.0
    #qdd[1] = 0.0
    
    return np.multiply(time_scaling, [qd1, qd2, qdd[0][0], qdd[1][0]])
   
def update_state():
    global x # State variables
    global ti, Ts
   
    x0 = np.array(x)  # Current state is the initial state
                   # for an integration step       

    mv_out = u_fun()                            
                                
    # Ts = 0.01
    #Ts = time.perf_counter() - ti # elapsed CPU time... the time dedicated by the CPU to the process, aka execution time or processor time
    Ts = time.time() - ti # elapsed wall-clock time... the time it takes to run the process, aka elapsed time or running time
    #print("Sampling time Ts is %f" %Ts)
    
    Ts = 0.001
    t = np.linspace(0.0, Ts, 2)
    xt = odeint(xd_fun, x0, t)   # Perform integration using Fortran's LSODA (Adams & BDF methods)

    x = xt[-1,:]    # Gets the last row of x and all columns

    #check_state_bounds()
    
    #ti = time.perf_counter()
    ti = time.time()
    
def keyboard_logic(key):
    global mv_user
    
    # Actuator 1 - mv_user[0]
    if key == pygame.K_DOWN:
        mv_user[0] -= 0.1
        if mv_user[0] < -1.0:
            mv_user[0] = -1.0
            if auto == 0:
                print("Actuator 1 reached lower limit!")
            else:
                print("Mininum reference value reached for joint 1!")
        else:
            if auto == 0:
                print("Reducing actuator 1 torque to: %f [percent] (%f [Nm])" %(mv_user[0], mv_user[0]*MV_USER_TO_OUT))
            else:
                print("Decreasing joint 1 reference to: %f [percent] (%f [Nm])" %(mv_user[0], mv_user[0]*MV_USER_TO_REF))
    if key == pygame.K_UP:
        mv_user[0] += 0.1
        if mv_user[0] > 1.0:
            mv_user[0] = 1.0
            if auto == 0:
                print("Actuator 1 reached upper limit!")
            else:
                print("Maximum reference value reached for joint 1!")
        else:
            if auto == 0:
                print("Increasing actuator 1 torque to: %f [percent] (%f [Nm])" %(mv_user[0], mv_user[0]*MV_USER_TO_OUT))
            else:
                print("Increasing joint 1 reference to: %f [percent] (%f [Nm])" %(mv_user[0], mv_user[0]*MV_USER_TO_REF))
    # Actuator 2 - mv_user[1]
    if key == pygame.K_LEFT:
        mv_user[1] -= 0.1
        if mv_user[1] < -1.0:
            mv_user[1] = -1.0
            if auto == 0:
                print("Actuator 2 reached lower limit!")
            else:
                print("Mininum reference value reached for joint 2!")
        else:
            if auto == 0:
                print("Reducing actuator 2 torque to: %f [percent] (%f [Nm])" %(mv_user[1], mv_user[1]*MV_USER_TO_OUT))
            else:
                print("Decreasing joint 2 reference to: %f [percent] (%f [Nm])" %(mv_user[1], mv_user[1]*MV_USER_TO_REF))
    if key == pygame.K_RIGHT:
        mv_user[1] += 0.1
        if mv_user[1] > 1.0:
            mv_user[1] = 1.0
            if auto == 0:
                print("Actuator 2 reached upper limit!")
            else:
                print("Maximum reference value reached for joint 2!")
        else:
            if auto == 0:
                print("Increasing actuator 2 torque to: %f [percent] (%f [Nm])" %(mv_user[1], mv_user[1]*MV_USER_TO_OUT))
            else:
                print("Increasing joint 2 reference to: %f [percent] (%f [Nm])" %(mv_user[1], mv_user[1]*MV_USER_TO_REF))
                
def handle_keyboard():
    global time_scaling, auto, mv_user, ref, x
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # pygame.QUITis sent when the user clicks the window's "X" button, or when the system 'asks' for the process to quit
                                       # http://stackoverflow.com/questions/10080715/pygame-event-event-type-pygame-quit-confusion
            pygame.quit(); #sys.exit() if sys is imported
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            pygame.quit(); #sys.exit() if sys is imported
        if event.type == pygame.KEYDOWN:    # http://www.pygame.org/docs/ref/key.html
            if    event.key == pygame.K_UP \
               or event.key == pygame.K_DOWN \
               or event.key == pygame.K_LEFT \
               or event.key == pygame.K_RIGHT:
                keyboard_logic(event.key)
            if event.key == pygame.K_a:   
                auto = 1
                ref = x[0] # Make reference equal to the current state when
                            # switching from  manual to auto.
                mv_user = ref/MV_USER_TO_REF
                print("\nControl mode is set to [automatic]")
                print("Reference is %f" %ref)
            if event.key == pygame.K_m:
                auto = 0
                print("\nControl mode is set to [manual]")    
            if event.key == K_s:
                arrange_data()
                np.save("simdata.npy",data)
                print("File saved to 'simdata.npy'")
                #savetxt("simdata.csv",data)
                #print("File saved to 'simdata.csv'")
    return True

def init_display():
    global XMAX, YMAX, screen
    
    # Initialize PyGame and setup a PyGame display
    pygame.init()
    # pygame.display.set_mode()
    screen = pygame.display.set_mode((XMAX,YMAX))
    pygame.display.set_caption('Watertank Sim')
    pygame.key.set_repeat(1,50)     # Works with essentially no delay.
    #pygame.key.set_repeat(0,50)    # Doesn't work because when the delay
                                    # is set to zero, key.set_repeat is
                                    # returned to the default, disabled
                                    # state.
    
def main():
    global tw0, XMAX, YMAX, screen

    # Initialize time counter
    tw0 = time.time()
    
    init_display()
    

                             
    while True:

        handle_keyboard()
        update_state()
        update_display()
        store_data()

        #pygame.time.wait(10) # Set a wait-time to delay capture from keyboard to 10 miliseconds
                             # For very fast processes, it may be necessary to slow down the keyboard 
                             # capture rate in order to reduce fast/abrubpt responses. However, beware
                             # that this delay also reduces the sampling time of the simulator.
                             # Without this delay, the sampling time is on average 8 ms, with the
                             # 10 ms delay, the sampling time increase to 18 ms.
                
if __name__ == '__main__': main()