from math import *



# Hyperparameters (all lengths in meters)
L = 10  # Length of arena
W = 10  # Width of arena
d = 0.05    # wheel diameter
w = 0.09    # width of robot
delta_t = 0.1   # time step in seconds
x0 = 0
y0 = 0      # initial coordinates of robot


# run a full simulation
def simulate():
    pass


# simulate one time step
def simulate_step(state_k, input_k):
    '''
    Inputs:
    state_t: tuple of x, y, and theta at time k
    input_t: tuple of left wheel velocity and right wheel velocity

    Outputs:
    next_state: x, y, and theta at time k+1
    readings: sensor readings for l1, l2, big_omega, b1, and b2
    '''


    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    omegaR_k = input_k[0]
    omegaL_k = input_k[1]

    # state update
    x_next = x_k + (((d*delta_t/4)*cos(theta_k))*omegaR_k) - (((d*delta_t/4)*cos(theta_k))*omegaL_k)    #no noise component yet
    y_next = y_k + (((d*delta_t/4)*sin(theta_k))*omegaR_k) - (((d*delta_t/4)*sin(theta_k))*omegaL_k)    #no noise component yet
    theta_next = theta_k + ((d*delta_t/(2*w))*omegaR_k) + ((d*delta_t/(2*w))*omegaL_k)                  #no noise component yet

    # bound x between 0 and L
    if x_next < 0:
        x_next = 0
    elif x_next > L:
        x_next = L

    # bound y between 0 and R
    if y_next < 0:
        y_next = 0
    elif y_next > W:
        y_next = W

    # normalize theta between 0 and 2*pi
    if theta_next < 0:
        theta_next += 2*pi
    elif theta_next > 360:
        theta_next -= 2*pi

    # sensor readings



    pass



if __name__ == "__main__":
    simulate()