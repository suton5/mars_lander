#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

# masses, gravitational constant, initial position and velocity
# note that mars has a radius of 3390 km
m = 1
M = 6.42e24
G = 6.67e-11

# simulation time and timestep
t_max = 10000
dt = [0.1]

def orbit_grapher2(p, v):
    """Takes initial position and velocity vectors and outputs the orbital."""

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
            p_x = p[0] + interval * v[0]
            p_y = p[1] + interval * v[1]
            p_z = p[2] + interval * v[2]
            p = (p_x, p_y, p_z)
            v_x = v[0] + interval * a[0]
            v_y = v[1] + interval * a[1]
            v_z = v[2] + interval * a[2]
            v = (v_x, v_y, v_z)
        assert len(p_list) == len(v_list) #check if position list and velocity list have equal number of terms
        p_x_list = []
        p_y_list = []
        p_z_list = []
        for position_vector in p_list:
            p_x_list.append(position_vector[0])
            p_y_list.append(position_vector[1])
            p_z_list.append(position_vector[2])
            
        v_x_list = []
        v_y_list = []
        v_z_list = []
        for velocity_vector in v_list:
            v_x_list.append(velocity_vector[0])
            v_y_list.append(velocity_vector[1])
            v_z_list.append(velocity_vector[2])

        # convert trajectory lists into arrays, so they can be indexed more easily
        p_x_array = np.array(p_x_list)
        p_y_array = np.array(p_y_list)
        p_z_array = np.array(p_z_list)
        
        v_x_array = np.array(v_x_list)
        v_y_array = np.array(v_y_list)
        v_z_array = np.array(v_z_list)

        # plot the orbital graph in 3-D
        plt.figure(1)
        plt.clf()
        plt.xlabel('x')
        plt.ylabel ('y')
        plt.title('For dt=' + str(interval))
        plt.grid()
        plt.plot(p_x_array, p_y_array)
#       plt.plot(t_array, v_array, label='v (m/s)')
        plt.legend()
        plt.show()
    return 0

orbit_grapher2((-4.0e6, 0, 0), (0, 10347, 0)) #circular
orbit_grapher2((-4.0e6, 0, 0), (0, 8448, 0)) #elliptical
orbit_grapher2((-4.0e6, 0, 0), (0, 15000, 0)) #hyperbolic
