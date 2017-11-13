#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

# masses, gravitational constant, initial position and velocity
# note that mars has a radius of 3390 km
m = 1
M = 6.42e24
G = 6.67e-11

# simulation time and timestep
t_max = 300
dt = [0.1]

def orbit_grapher1(p, v):
    """Takes initial position and velocity vectors and outputs y as a function of t."""
    
    assert len(p) == 3, "Position vector not 3-dimensional"
    assert len(v) == 3, "Velocity vector not 3-dimensional"
    
    for interval in dt:

        t_array = np.arange(0, t_max, interval)

        # initialise empty lists to record trajectories. will end up in a list of tuples.
        p_list = []
        v_list = []

        # Euler integration
        for t in t_array:

            # append current state to trajectories
            p_list.append(p)
            v_list.append(v)

            # calculate new position and velocity
            r = np.linalg.norm(p)
            r_unit = (p[0]/r, p[1]/r, p[2]/r)
            F_mag = (G*M*m)/(r**2)
            F = (r_unit[0]*F_mag, r_unit[1]*F_mag, r_unit[2]*F_mag)
            a = (-F[0]/m, -F[1]/m, -F[2]/m)
            p = (p[0] + interval * v[0], p[1] + interval * v[1], p[2] + interval * v[2])
            v = (v[0] + interval * a[0], v[1] + interval * a[1], v[2] + interval * a[2])
        assert len(p_list) == len(v_list) #check if position list and velocity list have equal number of terms
        
        p_y_list = []
        for position_vector in p_list:
            p_y_list.append(position_vector[1])

        # convert trajectory list into arrays, so they can be indexed more easily
        p_y_array = np.array(p_y_list)

        # plot the orbital graph in 3-D
        plt.figure(1)
        plt.clf()
        plt.xlabel('t')
        plt.ylabel ('y')
        plt.title('For dt=' + str(interval))
        plt.grid()
        plt.plot(t_array, p_y_array)
#       plt.plot(t_array, v_array, label='v (m/s)')
        plt.legend()
        plt.show()
    return 0

orbit_grapher1((0, 4000000, 0), (0, 0, 0))
