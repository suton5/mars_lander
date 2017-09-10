#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = [0.1, 0.01, 1.0, 1.7]

for interval in dt:
    
    t_array = np.arange(0, t_max, interval)
    
    # initialise empty lists to record trajectories
    x_list = []
    v_list = []
    
    # calculate x and v for the next time instance (to use in Verlet)
    x_list.append(x)
    v_list.append(v)
    a = -k * x / m
    x = x + interval * v
    v = v + interval * a
    x_list.append(x)
    v_list.append(v)
    
    for i in range(len(t_array)-2):
        a = -k * x / m
        x = 2*(x_list[-1]) - (x_list[-2]) + (interval ** 2) * a
        v = (x - (x_list[-2])) / (2*interval)
        
        x_list.append(x)
        v_list.append(v)
        
    # convert trajectory lists into arrays, so they can be indexed more easily
    x_array = np.array(x_list)
    v_array = np.array(v_list)
 
    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.title('For dt=' + str(interval))
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()
