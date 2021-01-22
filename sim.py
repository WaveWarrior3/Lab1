from math import sin, cos, pi, atan



# Hyperparameters (all lengths in meters)
L = 10  # Length of arena
H = 10  # Width of arena
d = 0.05    # wheel diameter
w = 0.09    # width of robot
delta_t = 0.1   # time step in seconds
x0 = 0
y0 = 0      # initial coordinates of robot
B_x = 0     # Earth's magnetic field, replace later
B_y = 0


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




    # find regions for LIDAR readings
    regionF = None
    regionR = None


    if 0 <= theta_k <= atan((L-x_k)/(H-y_k)) or (3*pi/2 + atan((H-y_k)/x_k)) <= theta_k <= 2*pi:
        regionF = 1
    elif atan((L-x_k)/(H-y_k)) <= theta_k <= (pi/2 + atan(y_k/(L-x_k))):
        regionF = 2
    elif (pi/2 + atan(y_k/(L-x_k))) <= theta_k <= (pi + atan(x_k/y_k)):
        regionF = 3
    elif (pi + atan(x_k/y_k)) <= theta_k <= (3*pi/2 + atan((H-y_k)/x_k)):
        regionF = 4

    if regionF == None:
        print("Something has gone horribly wrong with Front LIDAR")
        return


    if 0 <= theta_k <= atan(y_k/(L-x_k)) or (3*pi/2 + atan((L-x_k)/(H-y_k))) <= theta_k <= 2*pi:
        regionR = 1
    elif atan(y_k/(L-x_k)) <= theta_k <= (pi/2 + atan(x_k/y_k)):
        regionR = 2
    elif (pi/2 + atan(x_k/y_k)) <= theta_k <= (pi + atan((H-y_k)/x_k)):
        regionR = 3
    elif (pi + atan((H-y_k)/x_k)) <= theta_k <= (3*pi/2 + atan((L-x_k)/(H-y_k))):
        regionR = 4

    if regionR == None:
        print("Something has gone horribly wrong with Right LIDAR")
        return



    # Take Readings
    l1_k = f1(state_k, regionF)
    l2_k = f2(state_k, regionR)
    big_omega_k = (d/(2*w))*(omegaR_k+omegaL_k)
    b1_k = B_x*cos(theta_k) - B_y*sin(theta_k)
    b2_k = B_x*sin(theta_k) + B_y*cos(theta_k)

    next_state = (x_next, y_next, theta_next)
    sensor_readings = (l1_k, l2_k, big_omega_k, b1_k, b2_k)

    return [next_state, sensor_readings]


# Front LIDAR function
def f1(state_k, regionF):
    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    if regionF == 1:
        return ((H-y_k)/cos(theta_k))
    elif regionF == 2:
        return ((L-x_k)/sin(theta_k))
    elif regionF == 3:
        return(-y_k/cos(theta_k))
    elif regionF == 4:
        return(-x_k/sin(theta_k))
    else:
        print("Bad Front Region")
        return



def f2(state_k, regionR):
    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    if regionR == 1:
        return ((L-x_k)/cos(theta_k))
    elif regionR == 2:
        return (y_k/sin(theta_k))
    elif regionR == 3:
        return(-x_k/cos(theta_k))
    elif regionR == 4:
        return(-(H-y_k)/sin(theta_k))
    else:
        print("Bad Right Region")
        return

if __name__ == "__main__":
    curr_state = (0, 0, 0)
    curr_input = (1, 0)

    next_state, sensor_readings = simulate_step(curr_state, curr_input)
    print(next_state)
    print(sensor_readings)