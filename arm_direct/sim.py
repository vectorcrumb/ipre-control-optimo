import os, sys, time
import pygame
import numpy as np


user_configs = {
    "k_lr_mod": 0.1,
    "k_ud_mod": 0.1,
    "u_sat": 1.0
}

user_control_variables = {
    "u_1": 0.0,
    "u_2": 0.0, 
}

def saturate(x,x_1,x_2=None):
    if x < x_1:
        return x_1
    elif x > x_2:
        return x_2
    return x

def saturate(x, x_lim):
    x_lim = np.abs(x_lim)
    if x < -x_lim:
        return -x_lim
    elif x > x_lim:
        return x_lim
    return x

def kb_handler(control_vars, configs):

    for event in pygame.event.get():
        # Quit simulation either by close button or ESC key
        if event.type == pygame.QUIT:
            pygame.quit()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            pygame.quit()
        # Handle keypresses
        if event.type == pygame.KEYDOWN:
            # Set control mode to automatic (closed loop)
            if event.key == pygame.K_a:
                pass
            # Set control mode to manual (open loop)
            if event.key == pygame.K_m:
                pass
            # Save current data to a log file
            if event.key == pygame.K_s:
                pass
            # Handle key presses for manual mode
            if event.key == pygame.K_UP:
                control_vars["u_1"] += configs["k_ud_mod"]
            if event.key == pygame.K_DOWN:
                control_vars["u_1"] -= configs["k_ud_mod"]
            if event.key == pygame.K_LEFT:
                control_vars["u_2"] -= configs["k_lr_mod"]
            if event.key == pygame.K_RIGHT:
                control_vars["u_2"] += configs["k_lr_mod"]
    control_vars["u_1"] = saturate(control_vars["u_1"], configs["u_sat"])
    control_vars["u_2"] = saturate(control_vars["u_2"], configs["u_sat"])
