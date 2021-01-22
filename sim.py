from math import sin, cos, pi, atan
import numpy as np
import matplotlib.pyplot as plt



# Hyperparameters (all lengths in meters)
L = 1  # Length of arena
H = 1  # Width of arena
d = 0.05    # wheel diameter
w = 0.09    # width of robot
delta_t = 0.1   # time step in seconds
x0 = 0
y0 = 0      # initial coordinates of robot
B_x = 0     # Earth's magnetic field, replace later
B_y = 1




# run a full simulation
def simulate(input_list, init_state):
    state_list = []
    sensor_list = []
    
    state = init_state
    for inputs in input_list:
        state_list.append(state)
        state, sensor = simulate_step(state, inputs)
        sensor_list.append(sensor)
    
    return state_list, sensor_list


# simulate one time step
def simulate_step(state_k, input_k):
    '''
    Inputs:
    state_t: tuple of x, y, and theta at time k, in that order
    input_t: tuple of left wheel velocity and right wheel velocity, in that order

    Outputs:
    next_state: x, y, and theta at time k+1
    readings: sensor readings for l1, l2, big_omega, b1, and b2
    '''


    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    omegaL_k = input_k[0]
    omegaR_k = input_k[1]



    # state update
    x_next = x_k + (-((d*delta_t/4)*sin(theta_k))*omegaR_k) + (((d*delta_t/4)*sin(theta_k))*omegaL_k)    #no noise component yet
    y_next = y_k + (-((d*delta_t/4)*cos(theta_k))*omegaR_k) + (((d*delta_t/4)*cos(theta_k))*omegaL_k)    #no noise component yet
    theta_next = theta_k + ((d*delta_t/(2*w))*omegaR_k) + ((d*delta_t/(2*w))*omegaL_k)                  #no noise component yet

    # bound x between 0 and L
    if x_next < 0.0:
        x_next = 0.0
    elif x_next > L:
        x_next = L

    # bound y between 0 and H
    if y_next < 0:
        y_next = 0.0
    elif y_next > H:
        y_next = H

    # normalize theta between 0 and 2*pi
    if theta_next < 0.0:
        theta_next += 2*pi
    elif theta_next > 2*pi:
        theta_next -= 2*pi



    # Take Readings
    l1_k = L - x_k
    l2_k = H - y_k

    big_omega_k = (d/(2*w))*(omegaR_k+omegaL_k)
    if big_omega_k < 0.0:
        big_omega_k += 2*pi
    elif big_omega_k > 2*pi:
        big_omega_k -= 2*pi

    b1_k = B_x*cos(theta_k) - B_y*sin(theta_k)
    b2_k = B_x*sin(theta_k) + B_y*cos(theta_k)

    next_state = (x_next, y_next, theta_next)
    sensor_readings = (l1_k, l2_k, big_omega_k, b1_k, b2_k)

    return next_state, sensor_readings



def plot_path(state_list):
    x = []
    y = []

    for states in state_list:
        x.append(states[0])
        y.append(states[1])

    f = plt.figure(1)
    plt.xlim((0,L))
    plt.ylim((0,H))
    f.set_figheight(5)
    f.set_figwidth(5)
    plt.plot(x,y)
    plt.show()

    


if __name__ == "__main__":
    init_state = (0.5, 0.5, pi)
    input_list = []
    for i in np.arange(100):
        input_list.append((1.0, -0.2))

    for i in np.arange(100):
        input_list.append((1.0, -1.0))

    for i in np.arange(100):
        input_list.append((0.2, -1.0))

    for i in np.arange(500):
        input_list.append((1.0, -1.0))

    state_list, sensor_list = simulate(input_list, init_state)
    plot_path(state_list)